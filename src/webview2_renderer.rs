// WebView2 Offscreen Renderer
// Uses native WebView2 COM APIs with DirectComposition for hardware-accelerated rendering

use std::sync::{Arc, Mutex};
use anyhow::{Result, Context};
use tracing::{info, warn, error};

#[cfg(target_os = "windows")]
use windows::{
    core::*,
    Win32::{
        Foundation::*,
        System::Com::*,
        Graphics::Direct3D11::*,
        Graphics::Direct3D::*,
        Graphics::Dxgi::Common::*,
        UI::WindowsAndMessaging::*,
    },
};

use webview2_com::Microsoft::Web::WebView2::Win32::*;
use wgpu;

/// WebView2 offscreen browser with DirectComposition rendering
/// 
/// This implementation uses WebView2's composition mode to render directly to a
/// D3D11 texture, which we then import into wgpu for display in Slint.
pub struct WebView2OffscreenBrowser {
    /// WebView2 environment
    environment: Option<ICoreWebView2Environment>,
    
    /// WebView2 controller (manages the WebView lifecycle)
    controller: Option<ICoreWebView2Controller>,
    
    /// WebView2 core instance (handles navigation, events, etc.)
    webview: Option<ICoreWebView2>,
    
    /// Composition controller for offscreen rendering
    composition_controller: Option<ICoreWebView2CompositionController>,
    
    /// D3D11 device for texture sharing
    d3d_device: Option<ID3D11Device>,
    
    /// D3D11 device context
    d3d_context: Option<ID3D11DeviceContext>,
    
    /// Current rendered texture from WebView2
    current_texture: Arc<Mutex<Option<ID3D11Texture2D>>>,
    
    /// wgpu device for texture import
    wgpu_device: Arc<wgpu::Device>,
    
    /// wgpu queue for commands
    wgpu_queue: Arc<wgpu::Queue>,
    
    /// Window handle (hidden window for WebView2)
    hwnd: HWND,
    
    /// Current URL
    current_url: String,
    
    /// Browser dimensions
    width: u32,
    height: u32,
}

impl WebView2OffscreenBrowser {
    /// Create a new WebView2 offscreen browser instance
    /// 
    /// # Arguments
    /// * `url` - Initial URL to load
    /// * `width` - Browser viewport width
    /// * `height` - Browser viewport height
    /// * `wgpu_device` - wgpu device for texture import
    /// * `wgpu_queue` - wgpu queue for commands
    pub async fn new(
        url: &str,
        width: u32,
        height: u32,
        wgpu_device: Arc<wgpu::Device>,
        wgpu_queue: Arc<wgpu::Queue>,
    ) -> Result<Self> {
        info!("Creating WebView2 offscreen browser for URL: {}", url);
        info!("Dimensions: {}x{}", width, height);
        
        #[cfg(not(target_os = "windows"))]
        {
            anyhow::bail!("WebView2 is only supported on Windows");
        }
        
        #[cfg(target_os = "windows")]
        {
            // Initialize COM
            unsafe {
                CoInitializeEx(None, COINIT_APARTMENTTHREADED).ok();
            }
            
            // Create hidden window for WebView2
            let hwnd = Self::create_hidden_window(width, height)
                .context("Failed to create hidden window")?;
            
            info!("Created hidden window with HWND: {:?}", hwnd);
            
            // Create D3D11 device for texture sharing
            let (d3d_device, d3d_context) = Self::create_d3d11_device()
                .context("Failed to create D3D11 device")?;
            
            info!("Created D3D11 device for texture sharing");
            
            let current_texture = Arc::new(Mutex::new(None));
            let current_url = url.to_string();
            
            Ok(Self {
                environment: None,
                controller: None,
                webview: None,
                composition_controller: None,
                d3d_device: Some(d3d_device),
                d3d_context: Some(d3d_context),
                current_texture,
                wgpu_device,
                wgpu_queue,
                hwnd,
                current_url,
                width,
                height,
            })
        }
    }
    
    #[cfg(target_os = "windows")]
    fn create_hidden_window(width: u32, height: u32) -> Result<HWND> {
        unsafe {
            let class_name = w!("WebView2OffscreenWindow");
            
            let wc = WNDCLASSW {
                lpfnWndProc: Some(Self::window_proc),
                lpszClassName: class_name,
                ..Default::default()
            };
            
            let atom = RegisterClassW(&wc);
            if atom == 0 {
                return Err(anyhow::anyhow!("Failed to register window class"));
            }
            
            let hwnd = CreateWindowExW(
                WINDOW_EX_STYLE(0),
                class_name,
                w!("WebView2 Offscreen"),
                WS_OVERLAPPEDWINDOW,
                CW_USEDEFAULT,
                CW_USEDEFAULT,
                width as i32,
                height as i32,
                None,
                None,
                None,
                None,
            )?;
            
            // Keep window hidden
            ShowWindow(hwnd, SW_HIDE);
            
            Ok(hwnd)
        }
    }
    
    #[cfg(target_os = "windows")]
    unsafe extern "system" fn window_proc(
        hwnd: HWND,
        msg: u32,
        wparam: WPARAM,
        lparam: LPARAM,
    ) -> LRESULT {
        DefWindowProcW(hwnd, msg, wparam, lparam)
    }
    
    #[cfg(target_os = "windows")]
    fn create_d3d11_device() -> Result<(ID3D11Device, ID3D11DeviceContext)> {
        unsafe {
            let mut device: Option<ID3D11Device> = None;
            let mut context: Option<ID3D11DeviceContext> = None;
            
            D3D11CreateDevice(
                None, // Use default adapter
                D3D_DRIVER_TYPE_HARDWARE,
                None,
                D3D11_CREATE_DEVICE_BGRA_SUPPORT, // Required for DirectComposition
                None,
                D3D11_SDK_VERSION,
                Some(&mut device),
                None,
                Some(&mut context),
            ).map_err(|e| anyhow::anyhow!("D3D11CreateDevice failed: {}", e))?;
            
            let device = device.ok_or_else(|| anyhow::anyhow!("Failed to create D3D11 device"))?;
            let context = context.ok_or_else(|| anyhow::anyhow!("Failed to create D3D11 context"))?;
            
            Ok((device, context))
        }
    }
    
    /// Initialize the WebView2 runtime and create the browser instance
    /// 
    /// This must be called after construction to complete initialization.
    /// It's separate from `new()` because it requires async operations.
    pub async fn initialize(&mut self) -> Result<()> {
        info!("Initializing WebView2 runtime...");
        
        #[cfg(target_os = "windows")]
        {
            use webview2_com::Microsoft::Web::WebView2::Win32::CreateCoreWebView2EnvironmentCompletedHandler;
            use webview2_com::Microsoft::Web::WebView2::Win32::CreateCoreWebView2ControllerCompletedHandler;
            
            // Create WebView2 environment
            info!("Creating WebView2 environment...");
            
            // Use tokio channel for async completion
            let (env_tx, env_rx) = tokio::sync::oneshot::channel();
            
            let env_handler = CreateCoreWebView2EnvironmentCompletedHandler::create(Box::new(
                move |result, environment| {
                    if result.is_ok() {
                        if let Some(env) = environment {
                            info!("WebView2 environment created successfully");
                            let _ = env_tx.send(Ok(env));
                        } else {
                            let _ = env_tx.send(Err(anyhow::anyhow!("Environment is None")));
                        }
                    } else {
                        let _ = env_tx.send(Err(anyhow::anyhow!("Environment creation failed: {:?}", result)));
                    }
                    Ok(())
                }
            ));
            
            // Create environment with default settings
            unsafe {
                CreateCoreWebView2Environment(&env_handler)
                    .map_err(|e| anyhow::anyhow!("Failed to create WebView2 environment: {}", e))?;
            }
            
            // Wait for environment creation
            let environment = env_rx.await
                .map_err(|e| anyhow::anyhow!("Environment creation channel error: {}", e))??;
            
            info!("WebView2 environment ready");
            
            // Create composition controller
            info!("Creating WebView2 composition controller...");
            
            let (controller_tx, controller_rx) = tokio::sync::oneshot::channel();
            let hwnd = self.hwnd;
            
            let controller_handler = CreateCoreWebView2ControllerCompletedHandler::create(Box::new(
                move |result, controller| {
                    if result.is_ok() {
                        if let Some(ctrl) = controller {
                            info!("WebView2 controller created successfully");
                            let _ = controller_tx.send(Ok(ctrl));
                        } else {
                            let _ = controller_tx.send(Err(anyhow::anyhow!("Controller is None")));
                        }
                    } else {
                        let _ = controller_tx.send(Err(anyhow::anyhow!("Controller creation failed: {:?}", result)));
                    }
                    Ok(())
                }
            ));
            
            // Create composition controller
            unsafe {
                environment.CreateCoreWebView2CompositionController(hwnd, &controller_handler)
                    .map_err(|e| anyhow::anyhow!("Failed to create composition controller: {}", e))?;
            }
            
            // Wait for controller creation
            let controller = controller_rx.await
                .map_err(|e| anyhow::anyhow!("Controller creation channel error: {}", e))??;
            
            info!("WebView2 composition controller ready");
            
            // Get the CoreWebView2
            let webview = unsafe {
                controller.CoreWebView2()
                    .map_err(|e| anyhow::anyhow!("Failed to get CoreWebView2: {}", e))?
            };
            
            info!("Got CoreWebView2 instance");
            
            // Get composition controller interface
            let composition_controller: ICoreWebView2CompositionController = controller.cast()
                .map_err(|e| anyhow::anyhow!("Failed to cast to composition controller: {}", e))?;
            
            // Set bounds
            let mut bounds = RECT {
                left: 0,
                top: 0,
                right: self.width as i32,
                bottom: self.height as i32,
            };
            
            unsafe {
                controller.put_Bounds(&mut bounds)
                    .map_err(|e| anyhow::anyhow!("Failed to set bounds: {}", e))?;
            }
            
            info!("Set WebView2 bounds to {}x{}", self.width, self.height);
            
            // Make controller visible
            unsafe {
                controller.put_IsVisible(true.into())
                    .map_err(|e| anyhow::anyhow!("Failed to set visible: {}", e))?;
            }
            
            // Navigate to URL
            info!("Navigating to: {}", self.current_url);
            let url_wide: Vec<u16> = self.current_url.encode_utf16().chain(std::iter::once(0)).collect();
            
            unsafe {
                webview.Navigate(PCWSTR::from_raw(url_wide.as_ptr()))
                    .map_err(|e| anyhow::anyhow!("Failed to navigate: {}", e))?;
            }
            
            // Store instances
            self.environment = Some(environment);
            self.controller = Some(controller);
            self.webview = Some(webview);
            self.composition_controller = Some(composition_controller);
            
            info!("✅ WebView2 initialized successfully!");
            
            Ok(())
        }
        
        #[cfg(not(target_os = "windows"))]
        {
            Err(anyhow::anyhow!("WebView2 is only supported on Windows"))
        }
    }
    
    /// Navigate to a new URL
    pub fn navigate(&mut self, url: &str) -> Result<()> {
        info!("Navigating to: {}", url);
        self.current_url = url.to_string();
        
        // TODO: Implement navigation via ICoreWebView2::Navigate
        
        Ok(())
    }
    
    /// Capture the current frame as a wgpu texture
    /// 
    /// Returns a wgpu::Texture containing the rendered WebView2 content
    pub fn capture_frame(&self) -> Result<wgpu::Texture> {
        // TODO: Get D3D11 texture from composition controller
        // TODO: Import into wgpu using dx12 interop
        
        Err(anyhow::anyhow!("Frame capture not yet implemented"))
    }
    
    /// Send mouse move event
    pub fn send_mouse_move(&self, _x: f32, _y: f32) -> Result<()> {
        // TODO: Forward to ICoreWebView2Controller::MoveFocus or composition controller
        Ok(())
    }
    
    /// Send mouse click event
    pub fn send_mouse_click(&self, _x: f32, _y: f32, _button: MouseButton) -> Result<()> {
        // TODO: Forward to composition controller
        Ok(())
    }
    
    /// Resize the browser viewport
    pub fn resize(&mut self, width: u32, height: u32) -> Result<()> {
        info!("Resizing WebView2 to {}x{}", width, height);
        self.width = width;
        self.height = height;
        
        // TODO: Update controller bounds
        
        Ok(())
    }
}

#[derive(Debug, Clone, Copy)]
pub enum MouseButton {
    Left,
    Right,
    Middle,
}

impl Drop for WebView2OffscreenBrowser {
    fn drop(&mut self) {
        #[cfg(target_os = "windows")]
        unsafe {
            if !self.hwnd.is_invalid() {
                let _ = DestroyWindow(self.hwnd);
            }
            CoUninitialize();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[tokio::test]
    async fn test_webview2_creation() {
        // This test requires Windows and WebView2 runtime installed
        #[cfg(target_os = "windows")]
        {
            // Create minimal wgpu instance for testing
            let instance = wgpu::Instance::new(wgpu::InstanceDescriptor::default());
            let adapter = instance
                .request_adapter(&wgpu::RequestAdapterOptions::default())
                .await
                .unwrap();
            let (device, queue) = adapter
                .request_device(&wgpu::DeviceDescriptor::default(), None)
                .await
                .unwrap();
            
            let device = Arc::new(device);
            let queue = Arc::new(queue);
            
            let result = WebView2OffscreenBrowser::new(
                "https://example.com",
                800,
                600,
                device,
                queue,
            ).await;
            
            // Should at least create the structure (full init is separate)
            assert!(result.is_ok());
        }
    }
}
