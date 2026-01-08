// Native Slint WebView2 integration wrapper.
//
// This module is a fresh rebuild of the original implementation, which had
// fallen into disrepair. The goal is to provide a predictable, well-structured
// interface that the rest of the UI can depend on while we continue rebuilding
// the DirectComposition capture path.
//
// For now we expose the original public API (initialise, navigate, resize,
// pointer forwarding, and frame retrieval). The Windows backend creates a real
// WebView2 controller so navigation works, but the rendered output is currently
// a synthetic gradient that lets the UI exercise layout and refresh logic until
// the capture pipeline is restored.

use anyhow::{anyhow, bail, Context, Result};
use base64::engine::general_purpose::STANDARD as BASE64_STANDARD;
use base64::Engine;
use serde::Deserialize;
use serde_json::{json, Value};
use slint::private_unstable_api::re_exports::{KeyEvent, PointerEvent};
use std::sync::Arc;
use tracing::{info, warn};

/// Slint-friendly WebView2 surface. The struct itself remains platform neutral;
/// all Windows-specific pieces sit behind `cfg(target_os = "windows")` so the
/// module still compiles (with a runtime error) on other targets.
pub struct SlintWebView2 {
    width: u32,
    height: u32,
    current_url: String,
    initialized: bool,
    #[cfg(target_os = "windows")]
    state: Option<WindowsWebViewState>,
}

impl SlintWebView2 {
    /// Create a new WebView wrapper. Actual WebView2 initialisation happens in
    /// [`initialize`](Self::initialize) to avoid blocking construction.
    pub fn new(initial_url: &Arc<String>, width: u32, height: u32) -> Result<Self> {
        if width == 0 || height == 0 {
            bail!("WebView requires non-zero dimensions");
        }

        Ok(Self {
            width,
            height,
            current_url: (**initial_url).clone(),
            initialized: false,
            #[cfg(target_os = "windows")]
            state: None,
        })
    }

    /// Initialise WebView2 and create the backing controller. Safe to call more
    /// than once; subsequent calls are ignored.
    pub fn initialize(&mut self) -> Result<()> {
        if self.initialized {
            return Ok(());
        }

        #[cfg(target_os = "windows")]
        {
            let state = WindowsWebViewState::create(self.width, self.height, &self.current_url)?;
            self.state = Some(state);

            // Immediately resize to ensure correct initial dimensions
            if let Some(state) = &self.state {
                state.resize(self.width, self.height)?;
            }
        }

        #[cfg(not(target_os = "windows"))]
        {
            bail!("WebView2 integration is only available on Windows");
        }

        self.initialized = true;
        info!(
            "SlintWebView2 initialised ({}x{} -> {}) - size enforced",
            self.width, self.height, self.current_url
        );
        Ok(())
    }

    /// Navigate the hosted browser to the provided URL.
    pub fn navigate(&mut self, url: &str) -> Result<()> {
        self.current_url = url.to_string();

        #[cfg(target_os = "windows")]
        {
            if let Some(state) = &self.state {
                state.navigate(url)?;
            }
        }

        Ok(())
    }

    /// Resize the WebView viewport.
    pub fn resize(&mut self, width: u32, height: u32) -> Result<()> {
        if width == 0 || height == 0 {
            bail!("WebView resize requires non-zero dimensions");
        }

        info!(
            "Slint WebView resize requested: {}x{} -> {}x{}",
            self.width, self.height, width, height
        );

        self.width = width;
        self.height = height;

        #[cfg(target_os = "windows")]
        {
            if let Some(state) = &self.state {
                state.resize(width, height)?;
            }
        }

        Ok(())
    }

    /// Forward pointer events from Slint to WebView2. Pointer forwarding is a
    /// stub for now; it will be reinstated once the composition pipeline is
    /// rebuilt.
    pub fn handle_pointer_event(&self, event: &PointerEvent, x: f32, y: f32) -> Result<()> {
        #[cfg(target_os = "windows")]
        {
            if let Some(state) = &self.state {
                state.send_pointer(event, x, y)?;
            }
        }

        Ok(())
    }

    /// Get the current cursor position for crosshair display
    fn get_cursor_position(&self) -> (f32, f32) {
        #[cfg(target_os = "windows")]
        {
            if let Some(state) = &self.state {
                return *state.cursor_position.lock();
            }
        }
        (0.0, 0.0)
    }

    /// Forward scroll wheel deltas from Slint to WebView2.
    pub fn handle_scroll_event(&self, delta_x: f32, delta_y: f32, x: f32, y: f32) -> Result<()> {
        #[cfg(target_os = "windows")]
        {
            if let Some(state) = &self.state {
                state.send_scroll(delta_x, delta_y, x, y)?;
            }
        }

        Ok(())
    }

    /// Forward key events from Slint to WebView2. Returns true if the event
    /// was handled by the embedded browser.
    pub fn handle_key_event(&self, event: &KeyEvent) -> Result<bool> {
        #[cfg(target_os = "windows")]
        {
            if let Some(state) = &self.state {
                return state.send_key(event);
            }
        }
        Ok(false)
    }

    /// Retrieve the latest rendered frame as raw RGBA pixels with crosshair overlay.
    pub fn get_latest_frame(&self) -> Option<(Vec<u8>, u32, u32)> {
        let mut frame = {
            #[cfg(target_os = "windows")]
            {
                if let Some(state) = &self.state {
                    match state.capture_frame() {
                        Ok(frame) => Some(frame),
                        Err(err) => {
                            warn!("WebView2 frame capture failed: {err:?}");
                            None
                        }
                    }
                } else {
                    None
                }
            }
            #[cfg(not(target_os = "windows"))]
            None
        };

        if frame.is_none() {
            frame = Some(generate_test_pattern(self.width, self.height));
        }

        // Add crosshair overlay
        if let Some((mut pixels, width, height)) = frame {
            self.draw_crosshair(&mut pixels, width, height);
            Some((pixels, width, height))
        } else {
            None
        }
    }

    /// Draw a crosshair at the current cursor position
    fn draw_crosshair(&self, pixels: &mut [u8], width: u32, height: u32) {
        let (cursor_x, cursor_y) = self.get_cursor_position();

        // Convert cursor position to pixel coordinates, ensuring they're within bounds
        let px = (cursor_x as u32).min(width.saturating_sub(1));
        let py = (cursor_y as u32).min(height.saturating_sub(1));

        let crosshair_size = 30u32; // Size of crosshair arms - make bigger for visibility
        let crosshair_thickness = 3u32; // Thickness of crosshair lines - make thicker

        // Draw horizontal line (avoid overflow with proper bounds checking)
        let start_x = px.saturating_sub(crosshair_size);
        let end_x = (px + crosshair_size).min(width.saturating_sub(1));

        for x in start_x..=end_x {
            let start_y = py.saturating_sub(crosshair_thickness / 2);
            let end_y = (py + crosshair_thickness / 2).min(height.saturating_sub(1));

            for y in start_y..=end_y {
                let idx = ((y * width + x) * 4) as usize;
                if idx + 3 < pixels.len() {
                    // Bright green crosshair for better contrast
                    pixels[idx] = 0; // R
                    pixels[idx + 1] = 255; // G
                    pixels[idx + 2] = 0; // B
                    pixels[idx + 3] = 255; // A (opaque)
                }
            }
        }

        // Draw vertical line (avoid overflow with proper bounds checking)
        let start_y = py.saturating_sub(crosshair_size);
        let end_y = (py + crosshair_size).min(height.saturating_sub(1));

        for y in start_y..=end_y {
            let start_x = px.saturating_sub(crosshair_thickness / 2);
            let end_x = (px + crosshair_thickness / 2).min(width.saturating_sub(1));

            for x in start_x..=end_x {
                let idx = ((y * width + x) * 4) as usize;
                if idx + 3 < pixels.len() {
                    // Bright green crosshair for better contrast
                    pixels[idx] = 0; // R
                    pixels[idx + 1] = 255; // G
                    pixels[idx + 2] = 0; // B
                    pixels[idx + 3] = 255; // A (opaque)
                }
            }
        }
    }
}

fn generate_test_pattern(width: u32, height: u32) -> (Vec<u8>, u32, u32) {
    let w = width.max(1);
    let h = height.max(1);
    let mut pixels = vec![0u8; (w * h * 4) as usize];

    for y in 0..h {
        for x in 0..w {
            let idx = ((y * w + x) * 4) as usize;
            pixels[idx] = ((x * 255) / w) as u8; // R gradient
            pixels[idx + 1] = ((y * 255) / h) as u8; // G gradient
            pixels[idx + 2] = 180; // Fixed B component
            pixels[idx + 3] = 255; // Alpha
        }
    }

    (pixels, w, h)
}

// -------------------------------------------------------------------------------------------------
// Windows implementation details
// -------------------------------------------------------------------------------------------------

#[cfg(target_os = "windows")]
use std::{
    cell::{Cell, RefCell},
    slice,
    sync::mpsc,
    time::{Duration, Instant},
};

#[cfg(target_os = "windows")]
use parking_lot::Mutex;

#[cfg(target_os = "windows")]
use webview2_com::{
    wait_with_pump, CallDevToolsProtocolMethodCompletedHandler, CoreWebView2EnvironmentOptions,
    CreateCoreWebView2CompositionControllerCompletedHandler,
    CreateCoreWebView2EnvironmentCompletedHandler,
    Microsoft::Web::WebView2::Win32::{
        CreateCoreWebView2EnvironmentWithOptions, ICoreWebView2,
        ICoreWebView2CompositionController, ICoreWebView2Controller, ICoreWebView2Environment3,
        ICoreWebView2EnvironmentOptions,
    },
};

#[cfg(target_os = "windows")]
use windows::{
    core::{Error, Interface, HSTRING, PCWSTR},
    Win32::{
        Foundation::{E_FAIL, HWND, LPARAM, LRESULT, RECT, WPARAM},
        Graphics::{
            Direct3D::D3D_DRIVER_TYPE_HARDWARE,
            Direct3D11::{
                D3D11CreateDevice, ID3D11Device, ID3D11DeviceContext, ID3D11Texture2D,
                D3D11_CPU_ACCESS_READ, D3D11_CREATE_DEVICE_BGRA_SUPPORT, D3D11_MAPPED_SUBRESOURCE,
                D3D11_MAP_READ, D3D11_SDK_VERSION, D3D11_TEXTURE2D_DESC, D3D11_USAGE_STAGING,
            },
            DirectComposition::{
                DCompositionCreateDevice, IDCompositionDevice, IDCompositionTarget,
                IDCompositionVisual,
            },
            Dxgi::{IDXGIDevice, IDXGISwapChain1},
        },
        System::LibraryLoader::GetModuleHandleW,
        UI::WindowsAndMessaging::{
            CreateWindowExW, DefWindowProcW, DestroyWindow, RegisterClassW, ShowWindow, CS_HREDRAW,
            CS_VREDRAW, CW_USEDEFAULT, SW_HIDE, WINDOW_EX_STYLE, WNDCLASSW, WS_OVERLAPPEDWINDOW,
        },
    },
};

#[cfg(target_os = "windows")]
struct DirectCompositionState {
    d3d_device: ID3D11Device,
    d3d_context: ID3D11DeviceContext,
    dcomp_device: IDCompositionDevice,
    _dcomp_target: IDCompositionTarget,
    dcomp_visual: IDCompositionVisual,
    swap_chain: Option<IDXGISwapChain1>,
    staging_texture: Option<ID3D11Texture2D>,
    width: u32,
    height: u32,
}

#[cfg(target_os = "windows")]
impl DirectCompositionState {
    fn dimensions(&self) -> (u32, u32) {
        (self.width, self.height)
    }

    fn new(hwnd: HWND, width: u32, height: u32) -> Result<Self> {
        unsafe {
            let mut device = None;
            let mut context = None;

            D3D11CreateDevice(
                None,
                D3D_DRIVER_TYPE_HARDWARE,
                None,
                D3D11_CREATE_DEVICE_BGRA_SUPPORT,
                None,
                D3D11_SDK_VERSION,
                Some(&mut device),
                None,
                Some(&mut context),
            )
            .map_err(|e| anyhow!("D3D11CreateDevice failed: {e:?}"))?;

            let device = device.ok_or_else(|| anyhow!("D3D11CreateDevice returned null device"))?;
            let context =
                context.ok_or_else(|| anyhow!("D3D11CreateDevice returned null context"))?;

            let dxgi_device: IDXGIDevice = device.cast()?;
            let dcomp_device: IDCompositionDevice = DCompositionCreateDevice(&dxgi_device)?;
            let dcomp_target = dcomp_device.CreateTargetForHwnd(hwnd, true)?;
            let dcomp_visual = dcomp_device.CreateVisual()?;
            dcomp_target.SetRoot(&dcomp_visual)?;
            dcomp_device.Commit()?;

            Ok(Self {
                d3d_device: device,
                d3d_context: context,
                dcomp_device,
                _dcomp_target: dcomp_target,
                dcomp_visual,
                swap_chain: None,
                staging_texture: None,
                width,
                height,
            })
        }
    }

    fn attach_to_webview(
        &self,
        composition_controller: &ICoreWebView2CompositionController,
    ) -> Result<()> {
        unsafe {
            composition_controller.SetRootVisualTarget(&self.dcomp_visual)?;
            self.dcomp_device.Commit()?;
        }
        Ok(())
    }

    fn ensure_swap_chain(&mut self) -> Result<()> {
        Err(anyhow!(
            "DirectComposition swap chain acquisition not available in this build"
        ))
    }

    fn capture_rgba(&mut self) -> Result<(Vec<u8>, u32, u32)> {
        unsafe {
            self.ensure_swap_chain()?;

            let swap_chain = self
                .swap_chain
                .as_ref()
                .ok_or_else(|| anyhow!("WebView composition swap chain unavailable"))?;

            let buffer: ID3D11Texture2D = swap_chain.GetBuffer(0)?;
            let mut desc = D3D11_TEXTURE2D_DESC::default();
            buffer.GetDesc(&mut desc);

            let width = desc.Width;
            let height = desc.Height;

            if self.staging_texture.is_none() || self.width != width || self.height != height {
                desc.BindFlags = 0;
                desc.MiscFlags = 0;
                desc.CPUAccessFlags = D3D11_CPU_ACCESS_READ.0 as u32;
                desc.Usage = D3D11_USAGE_STAGING;
                desc.SampleDesc.Count = 1;
                desc.SampleDesc.Quality = 0;

                let mut staging = None;
                self.d3d_device
                    .CreateTexture2D(&desc, None, Some(&mut staging))?;
                let staging = staging.ok_or_else(|| anyhow!("CreateTexture2D returned NULL"))?;
                self.staging_texture = Some(staging);
                self.width = width;
                self.height = height;
            }

            let staging = self
                .staging_texture
                .as_ref()
                .ok_or_else(|| anyhow!("Staging texture allocation failed"))?;

            self.d3d_context.CopyResource(staging, &buffer);
            self.d3d_context.Flush();

            let mut mapped = D3D11_MAPPED_SUBRESOURCE::default();
            self.d3d_context
                .Map(staging, 0, D3D11_MAP_READ, 0, Some(&mut mapped))?;

            let result = (|| {
                let row_pitch = mapped.RowPitch as usize;
                let src_base = mapped.pData as *const u8;
                let width_usize = width as usize;
                let height_usize = height as usize;
                let mut pixels = vec![0u8; width_usize * height_usize * 4];

                for y in 0..height_usize {
                    let src_row_ptr = src_base.add(y * row_pitch);
                    let src_row = slice::from_raw_parts(src_row_ptr, row_pitch);
                    let dst_row = &mut pixels[y * width_usize * 4..(y + 1) * width_usize * 4];

                    for x in 0..width_usize {
                        let src_offset = x * 4;
                        let dst_offset = x * 4;
                        let b = src_row[src_offset];
                        let g = src_row[src_offset + 1];
                        let r = src_row[src_offset + 2];
                        let a = src_row[src_offset + 3];
                        dst_row[dst_offset] = r;
                        dst_row[dst_offset + 1] = g;
                        dst_row[dst_offset + 2] = b;
                        dst_row[dst_offset + 3] = a;
                    }
                }

                Ok((pixels, width, height))
            })();

            self.d3d_context.Unmap(staging, 0);

            result
        }
    }

    fn resize(&mut self, width: u32, height: u32) -> Result<()> {
        if width == 0 || height == 0 {
            bail!("WebView resize requires non-zero dimensions");
        }

        if self.width == width && self.height == height {
            return Ok(());
        }

        unsafe {
            self.staging_texture = None;
            self.swap_chain = None;
            self.width = width;
            self.height = height;
            self.dcomp_device.Commit()?;
        }

        Ok(())
    }
}

#[cfg(target_os = "windows")]
#[derive(Clone, Copy, Debug)]
struct ViewportMetrics {
    client_width: f64,
    client_height: f64,
    scale: f64,
    device_pixel_ratio: f64,
    offset_left: f64,
    offset_top: f64,
    page_x: f64,
    page_y: f64,
    visual_width: f64,
    visual_height: f64,
    timestamp: Instant,
}

#[cfg(target_os = "windows")]
#[derive(Clone, Copy, Debug)]
struct DomViewportMetrics {
    width: f64,
    height: f64,
    offset_left: f64,
    offset_top: f64,
    page_x: f64,
    page_y: f64,
    scale: f64,
    device_pixel_ratio: f64,
    timestamp: Instant,
}

struct WindowsWebViewState {
    _environment: ICoreWebView2Environment3,
    controller: ICoreWebView2Controller,
    _composition_controller: ICoreWebView2CompositionController,
    webview: ICoreWebView2,
    hwnd: HWND,
    composition: RefCell<DirectCompositionState>,
    last_frame: Mutex<Option<(Vec<u8>, u32, u32)>>,
    last_frame_timestamp: Cell<Instant>,
    devtools_capture_count: Cell<u64>,
    capture_window_start: Cell<Instant>,
    pointer_buttons: Mutex<u32>,
    cursor_position: Mutex<(f32, f32)>,
    key_modifiers: Mutex<i32>,
    direct_capture_disabled: Cell<bool>,
    direct_capture_failure_reason: RefCell<Option<String>>,
    direct_capture_retry_after: Cell<Option<Instant>>,
    expected_width: Cell<u32>,
    expected_height: Cell<u32>,
    initial_url: RefCell<Option<String>>,
    navigation_completed: Cell<bool>,
    viewport_metrics: RefCell<Option<ViewportMetrics>>,
    dom_viewport_metrics: RefCell<Option<DomViewportMetrics>>,
}

#[cfg(target_os = "windows")]
impl WindowsWebViewState {
    fn create(width: u32, height: u32, url: &str) -> Result<Self> {
        let environment = create_environment()?;
        let hwnd = create_hidden_window(width, height)?;
        let composition = RefCell::new(DirectCompositionState::new(hwnd, width, height)?);
        let (controller, composition_controller) = create_controller(&environment, hwnd)?;
        composition
            .borrow()
            .attach_to_webview(&composition_controller)?;
        let bounds = RECT {
            left: 0,
            top: 0,
            right: width as i32,
            bottom: height as i32,
        };
        unsafe {
            controller.SetBoundsAndZoomFactor(bounds, 1.0)?;
            controller.SetIsVisible(true)?;
        }
        let webview = unsafe { controller.CoreWebView2()? };
        let pointer_buttons = Mutex::new(0);

        // Don't navigate immediately - wait for Slint to provide correct size first
        // Navigation will happen on first resize from Slint

        call_devtools_method(&webview, "Page.enable", "{}")?;

        // Enable input event processing so DevTools events are actually handled
        call_devtools_method(
            &webview,
            "Input.setIgnoreInputEvents",
            r#"{"ignore":false}"#,
        )?;

        let state = Self {
            _environment: environment,
            controller,
            _composition_controller: composition_controller,
            webview,
            hwnd,
            composition,
            last_frame: Mutex::new(None),
            last_frame_timestamp: Cell::new(Instant::now()),
            devtools_capture_count: Cell::new(0),
            capture_window_start: Cell::new(Instant::now()),
            pointer_buttons,
            cursor_position: Mutex::new((0.0, 0.0)),
            key_modifiers: Mutex::new(0),
            direct_capture_disabled: Cell::new(false),
            direct_capture_failure_reason: RefCell::new(None),
            direct_capture_retry_after: Cell::new(None),
            expected_width: Cell::new(width),
            expected_height: Cell::new(height),
            initial_url: RefCell::new(Some(url.to_string())),
            navigation_completed: Cell::new(false),
            viewport_metrics: RefCell::new(None),
            dom_viewport_metrics: RefCell::new(None),
        };

        // Ensure WebView has correct size immediately after creation
        // This prevents the initial stretched display issue
        state.resize(width, height)?;

        Ok(state)
    }

    fn navigate(&self, url: &str) -> Result<()> {
        if self.navigation_completed.get() {
            // Normal navigation - WebView is already properly sized
            let hurl = HSTRING::from(url);
            unsafe {
                self.webview.Navigate(&hurl)?;
            }
        } else {
            // Defer navigation until first resize - just update the URL
            *self.initial_url.borrow_mut() = Some(url.to_string());
        }
        Ok(())
    }

    fn resize(&self, width: u32, height: u32) -> Result<()> {
        // Update expected dimensions for size checking
        self.expected_width.set(width);
        self.expected_height.set(height);

        // Perform initial navigation if this is the first resize from Slint
        // This ensures the page loads with the correct dimensions from the start
        if !self.navigation_completed.get() {
            if let Some(url) = self.initial_url.borrow_mut().take() {
                info!(
                    "Performing deferred navigation to {} at correct size {}x{}",
                    url, width, height
                );
                unsafe {
                    let hurl = HSTRING::from(url);
                    self.webview.Navigate(&hurl)?;
                }
                self.navigation_completed.set(true);
            }
        }

        let previous_dimensions = {
            let composition = self.composition.borrow();
            composition.dimensions()
        };

        info!(
            "WebView resize: {}x{} -> {}x{}",
            previous_dimensions.0, previous_dimensions.1, width, height
        );

        {
            let mut composition = self.composition.borrow_mut();
            composition.resize(width, height)?;
        }

        let bounds = RECT {
            left: 0,
            top: 0,
            right: width as i32,
            bottom: height as i32,
        };

        unsafe {
            self.controller.SetBoundsAndZoomFactor(bounds, 1.0)?;
        }

        {
            let mut cache = self.last_frame.lock();
            *cache = None;
        }

        if previous_dimensions != (width, height) {
            if self.direct_capture_disabled.get() {
                self.direct_capture_retry_after
                    .set(Some(Instant::now() + Duration::from_secs(30)));
            } else {
                self.direct_capture_retry_after.set(None);
            }
        }

        Ok(())
    }

    /// Ensure the WebView has the correct size before frame capture
    /// This fixes the initial squished display issue by verifying dimensions
    fn ensure_correct_size(&self) -> Result<()> {
        let expected_size = (self.expected_width.get(), self.expected_height.get());
        let actual_size = {
            let composition = self.composition.borrow();
            composition.dimensions()
        };

        // If sizes don't match, trigger a resize to fix the squished display
        if actual_size != expected_size && expected_size.0 > 0 && expected_size.1 > 0 {
            warn!("Size mismatch detected during frame capture: actual {}x{}, expected {}x{} - correcting immediately", 
                  actual_size.0, actual_size.1, expected_size.0, expected_size.1);
            self.resize(expected_size.0, expected_size.1)?;
        }

        Ok(())
    }

    fn capture_frame(&self) -> Result<(Vec<u8>, u32, u32)> {
        // Ensure WebView size is correct before capturing any frame
        // This fixes the initial squished display issue
        self.ensure_correct_size()?;

        // Only clear cache when truly idle (no frame requests for extended period)
        // This prevents cache clearing during normal operation
        let now = Instant::now();
        let last_timestamp = self.last_frame_timestamp.get();
        if now.duration_since(last_timestamp) > Duration::from_secs(120) {
            // 2 minutes idle
            let mut cache = self.last_frame.lock();
            if cache.is_some() {
                *cache = None;
                drop(cache);
            }
        }

        if self.direct_capture_disabled.get() {
            if let Some(deadline) = self.direct_capture_retry_after.get() {
                if Instant::now() < deadline {
                    return self.capture_via_devtools_or_cache();
                }
            } else {
                return self.capture_via_devtools_or_cache();
            }

            self.direct_capture_disabled.set(false);
            self.direct_capture_failure_reason.borrow_mut().take();
            self.direct_capture_retry_after.set(None);
        }

        let capture_attempt = {
            let mut composition = self.composition.borrow_mut();
            composition.capture_rgba()
        };

        match capture_attempt {
            Ok(frame) => {
                if frame.0.iter().all(|&byte| byte == 0) {
                    self.disable_direct_capture(&format!(
                        "DirectComposition capture produced blank frame ({}x{}).",
                        frame.1, frame.2
                    ));
                    return self.capture_via_devtools_or_cache();
                }

                let mut cache = self.last_frame.lock();
                *cache = Some(frame.clone());
                Ok(frame)
            }
            Err(err) => {
                self.disable_direct_capture(&format!("DirectComposition capture failed: {err:?}"));
                match self.capture_via_devtools_or_cache() {
                    Ok(frame) => Ok(frame),
                    Err(fallback_err) => Err(err.context(format!(
                        "DevTools screenshot fallback also failed: {fallback_err:?}"
                    ))),
                }
            }
        }
    }

    fn capture_via_devtools_or_cache(&self) -> Result<(Vec<u8>, u32, u32)> {
        // Much more permissive limits - focus on preventing memory leaks, not restricting captures
        const MAX_DEVTOOLS_CAPTURES_PER_MINUTE: u64 = 1200; // Allow 20 captures per second
        const MIN_FRAME_INTERVAL_MS: u64 = 50; // Minimum 50ms between new captures (max 20 FPS actual DevTools calls)

        let now = Instant::now();
        let last_timestamp = self.last_frame_timestamp.get();
        let capture_count = self.devtools_capture_count.get();

        // Reset capture count every minute using a sliding window
        let window_start = self.capture_window_start.get();
        if now.duration_since(window_start) > Duration::from_secs(60) {
            self.capture_window_start.set(now);
            self.devtools_capture_count.set(0);
        }

        // Always try to serve from cache first if available and recent enough
        let time_since_last = now.duration_since(last_timestamp);
        if let Some(cached) = self.last_frame.lock().as_ref().cloned() {
            // Use cached frame if very recent (to maintain 30 FPS with fewer DevTools calls)
            if time_since_last < Duration::from_millis(MIN_FRAME_INTERVAL_MS) {
                // Debug: trace cache reuse frequency (only log occasionally)
                static mut CACHE_REUSE_COUNT: u64 = 0;
                unsafe {
                    CACHE_REUSE_COUNT += 1;
                    if CACHE_REUSE_COUNT % 300 == 0 {
                        // Log every 300 cache reuses (~10 seconds at 30 FPS)
                        info!(
                            "Reusing cached frame (count: {}, time_since_last: {}ms)",
                            CACHE_REUSE_COUNT,
                            time_since_last.as_millis()
                        );
                    }
                }
                return Ok(cached);
            }
            // Also use cached if we're near rate limit
            if capture_count >= MAX_DEVTOOLS_CAPTURES_PER_MINUTE * 9 / 10 {
                // 90% of limit
                info!(
                    "Rate limit approaching, using cached frame (count: {})",
                    capture_count
                );
                return Ok(cached);
            }
        }

        match self.capture_via_devtools() {
            Ok(fallback) => {
                // Update tracking info
                self.last_frame_timestamp.set(now);
                self.devtools_capture_count.set(capture_count + 1);

                // Debug: log actual DevTools captures (less frequent than cache reuse)
                if (capture_count + 1) % 10 == 0 {
                    info!(
                        "DevTools capture #{} successful ({}x{})",
                        capture_count + 1,
                        fallback.1,
                        fallback.2
                    );
                }

                // Store in cache with memory bounds check
                let frame_size = fallback.0.len();
                if frame_size <= 10 * 1024 * 1024 {
                    // Max 10MB frame size
                    let mut cache = self.last_frame.lock();
                    *cache = Some(fallback.clone());
                }

                Ok(fallback)
            }
            Err(fallback_err) => {
                if let Some(cached) = self.last_frame.lock().as_ref().cloned() {
                    // Use cached frame when DevTools fails
                    Ok(cached)
                } else {
                    // No cached frame available - this is critical
                    warn!(
                        "No cached frame available and DevTools capture failed: {fallback_err:?}"
                    );
                    Err(fallback_err)
                }
            }
        }
    }

    fn capture_via_devtools(&self) -> Result<(Vec<u8>, u32, u32)> {
        #[derive(Deserialize)]
        struct ScreenshotResponse {
            data: String,
        }

        let payload = json!({
            "format": "png",
            "fromSurface": true,
        });

        let response = call_devtools_method_with_result(
            &self.webview,
            "Page.captureScreenshot",
            &payload.to_string(),
        )?;

        // Scope to ensure early deallocation of intermediate data
        let (rgba_data, width, height) = {
            let screenshot: ScreenshotResponse =
                serde_json::from_str(&response).with_context(|| {
                    format!("Failed to parse captureScreenshot response: {response}")
                })?;

            // Memory bounds check on base64 data
            if screenshot.data.len() > 20 * 1024 * 1024 {
                // Max 20MB base64 string
                return Err(anyhow::anyhow!(
                    "DevTools screenshot response too large: {} bytes",
                    screenshot.data.len()
                ));
            }

            let png_bytes = BASE64_STANDARD
                .decode(screenshot.data.as_bytes())
                .context("Failed to decode captureScreenshot PNG payload")?;

            // Memory bounds check on PNG data
            if png_bytes.len() > 15 * 1024 * 1024 {
                // Max 15MB PNG
                return Err(anyhow::anyhow!(
                    "DevTools PNG data too large: {} bytes",
                    png_bytes.len()
                ));
            }

            let image = image::load_from_memory(&png_bytes)
                .context("Failed to decode PNG returned by captureScreenshot")?;
            let rgba = image.to_rgba8();
            let width = rgba.width();
            let height = rgba.height();

            // Final memory bounds check on raw RGBA data
            let rgba_data = rgba.into_raw();
            if rgba_data.len() > 12 * 1024 * 1024 {
                // Max 12MB RGBA
                return Err(anyhow::anyhow!(
                    "DevTools RGBA frame too large: {} bytes",
                    rgba_data.len()
                ));
            }

            (rgba_data, width, height)
        }; // All intermediate data (base64, PNG, image) deallocated here

        Ok((rgba_data, width, height))
    }

    fn disable_direct_capture(&self, reason: &str) {
        if !self.direct_capture_disabled.get() {
            warn!("Disabling DirectComposition capture for this session: {reason}");
            self.direct_capture_disabled.set(true);
            *self.direct_capture_failure_reason.borrow_mut() = Some(reason.to_string());
            self.direct_capture_retry_after
                .set(Some(Instant::now() + Duration::from_secs(30)));
        }
    }

    /// Execute JavaScript code in the WebView2
    fn execute_script(&self, js_code: &str) -> Result<()> {
        use windows::core::HSTRING;

        let js_string = HSTRING::from(js_code);
        unsafe {
            self.webview.ExecuteScript(&js_string, None)?;
        }
        Ok(())
    }

    fn get_viewport_metrics(
        &self,
        fallback_width: f64,
        fallback_height: f64,
    ) -> Result<ViewportMetrics> {
        const CACHE_DURATION: Duration = Duration::from_millis(250);

        if let Some(metrics) = *self.viewport_metrics.borrow() {
            if metrics.timestamp.elapsed() <= CACHE_DURATION {
                return Ok(metrics);
            }
        }

        let response =
            call_devtools_method_with_result(&self.webview, "Page.getLayoutMetrics", "{}")?;
        let value: Value = serde_json::from_str(&response)?;
        let visual = value.get("visualViewport").and_then(Value::as_object);
        let layout = value.get("layoutViewport").and_then(Value::as_object);

        let offset_left = visual
            .and_then(|v| v.get("offsetLeft"))
            .and_then(Value::as_f64)
            .or_else(|| visual.and_then(|v| v.get("pageX")).and_then(Value::as_f64))
            .unwrap_or(0.0);

        let offset_top = visual
            .and_then(|v| v.get("offsetTop"))
            .and_then(Value::as_f64)
            .or_else(|| visual.and_then(|v| v.get("pageY")).and_then(Value::as_f64))
            .unwrap_or(0.0);

        let page_x = layout
            .and_then(|v| v.get("pageX"))
            .and_then(Value::as_f64)
            .unwrap_or(0.0);

        let page_y = layout
            .and_then(|v| v.get("pageY"))
            .and_then(Value::as_f64)
            .unwrap_or(0.0);

        let client_width = visual
            .and_then(|v| v.get("clientWidth"))
            .and_then(Value::as_f64)
            .or_else(|| {
                layout
                    .and_then(|v| v.get("clientWidth"))
                    .and_then(Value::as_f64)
            })
            .unwrap_or(fallback_width);

        let client_height = visual
            .and_then(|v| v.get("clientHeight"))
            .and_then(Value::as_f64)
            .or_else(|| {
                layout
                    .and_then(|v| v.get("clientHeight"))
                    .and_then(Value::as_f64)
            })
            .unwrap_or(fallback_height);

        let visual_width = visual
            .and_then(|v| v.get("width"))
            .and_then(Value::as_f64)
            .unwrap_or(client_width);

        let visual_height = visual
            .and_then(|v| v.get("height"))
            .and_then(Value::as_f64)
            .unwrap_or(client_height);

        let scale = visual
            .and_then(|v| v.get("scale"))
            .and_then(Value::as_f64)
            .unwrap_or(1.0);

        let device_pixel_ratio = value
            .get("layoutViewport")
            .and_then(Value::as_object)
            .and_then(|v| v.get("scale"))
            .and_then(Value::as_f64)
            .unwrap_or(scale);

        let metrics = ViewportMetrics {
            client_width,
            client_height,
            scale,
            device_pixel_ratio,
            offset_left,
            offset_top,
            page_x,
            page_y,
            visual_width,
            visual_height,
            timestamp: Instant::now(),
        };

        *self.viewport_metrics.borrow_mut() = Some(metrics);

        Ok(metrics)
    }

    fn get_dom_viewport_metrics(
        &self,
        fallback_width: f64,
        fallback_height: f64,
    ) -> Result<DomViewportMetrics> {
        const CACHE_DURATION: Duration = Duration::from_millis(120);

        if let Some(metrics) = *self.dom_viewport_metrics.borrow() {
            if metrics.timestamp.elapsed() <= CACHE_DURATION {
                return Ok(metrics);
            }
        }

        let payload = json!({
            "expression": r#"(function() {
                try {
                    const vv = window.visualViewport;
                    const doc = document.documentElement;
                    const width = vv ? vv.width : window.innerWidth;
                    const height = vv ? vv.height : window.innerHeight;
                    const offsetLeft = vv ? vv.offsetLeft : 0;
                    const offsetTop = vv ? vv.offsetTop : 0;
                    const scale = vv ? vv.scale : 1;
                    const pageX = (typeof window.scrollX === 'number' ? window.scrollX : (doc && doc.scrollLeft) || 0);
                    const pageY = (typeof window.scrollY === 'number' ? window.scrollY : (doc && doc.scrollTop) || 0);
                    const devicePixelRatio = window.devicePixelRatio || 1;
                    return { width, height, offsetLeft, offsetTop, pageX, pageY, scale, devicePixelRatio };
                } catch (err) {
                    return { width: undefined, height: undefined, offsetLeft: 0, offsetTop: 0, pageX: 0, pageY: 0, scale: 1, devicePixelRatio: window.devicePixelRatio || 1 };
                }
            })()"#,
            "returnByValue": true,
            "awaitPromise": true,
        });

        let response = call_devtools_method_with_result(
            &self.webview,
            "Runtime.evaluate",
            &payload.to_string(),
        )?;

        let value: Value = serde_json::from_str(&response)?;
        if value.get("exceptionDetails").is_some() {
            tracing::debug!("Runtime.evaluate reported exception while fetching DOM viewport metrics: {response}");
        }

        let dom_metrics = if let Some(obj) = value
            .get("result")
            .and_then(|v| v.get("value"))
            .and_then(Value::as_object)
        {
            DomViewportMetrics {
                width: obj
                    .get("width")
                    .and_then(Value::as_f64)
                    .unwrap_or(fallback_width),
                height: obj
                    .get("height")
                    .and_then(Value::as_f64)
                    .unwrap_or(fallback_height),
                offset_left: obj.get("offsetLeft").and_then(Value::as_f64).unwrap_or(0.0),
                offset_top: obj.get("offsetTop").and_then(Value::as_f64).unwrap_or(0.0),
                page_x: obj.get("pageX").and_then(Value::as_f64).unwrap_or(0.0),
                page_y: obj.get("pageY").and_then(Value::as_f64).unwrap_or(0.0),
                scale: obj
                    .get("scale")
                    .and_then(Value::as_f64)
                    .filter(|scale| scale.is_finite() && *scale > 0.0)
                    .unwrap_or(1.0),
                device_pixel_ratio: obj
                    .get("devicePixelRatio")
                    .and_then(Value::as_f64)
                    .filter(|dpr| dpr.is_finite() && *dpr > 0.0)
                    .unwrap_or(1.0),
                timestamp: Instant::now(),
            }
        } else {
            tracing::debug!("DOM viewport metrics missing value payload: {response}");
            DomViewportMetrics {
                width: fallback_width,
                height: fallback_height,
                offset_left: 0.0,
                offset_top: 0.0,
                page_x: 0.0,
                page_y: 0.0,
                scale: 1.0,
                device_pixel_ratio: 1.0,
                timestamp: Instant::now(),
            }
        };

        *self.dom_viewport_metrics.borrow_mut() = Some(dom_metrics);

        Ok(dom_metrics)
    }

    fn send_pointer(&self, event: &PointerEvent, x: f32, y: f32) -> Result<()> {
        use slint::private_unstable_api::re_exports::{PointerEventButton, PointerEventKind};

        // Update cursor position tracking
        {
            let mut cursor_pos = self.cursor_position.lock();
            *cursor_pos = (x, y);
        }

        // Debug coordinate mapping - get dimensions from composition state
        if matches!(event.kind, PointerEventKind::Down) {
            let (w, h) = self.composition.borrow().dimensions();
            info!("WebView2 pointer event: kind={:?} button={:?} pos=({:.1}, {:.1}) webview_size=({}, {})", 
                  event.kind, event.button, x, y, w, h);

            // Check if coordinates need scaling - WebView might expect different coordinate system
            let x_ratio = x / w as f32;
            let y_ratio = y / h as f32;
            info!(
                "Coordinate ratios: x_ratio={:.3}, y_ratio={:.3} (should be 0.0-1.0 range)",
                x_ratio, y_ratio
            );
        }

        fn button_mask(button: PointerEventButton) -> u32 {
            match button {
                PointerEventButton::Left => 1,
                PointerEventButton::Right => 2,
                PointerEventButton::Middle => 4,
                PointerEventButton::Back => 8,
                PointerEventButton::Forward => 16,
                PointerEventButton::Other => 0,
                _ => 0,
            }
        }

        fn button_name(button: PointerEventButton) -> &'static str {
            match button {
                PointerEventButton::Left => "left",
                PointerEventButton::Right => "right",
                PointerEventButton::Middle => "middle",
                PointerEventButton::Back => "back",
                PointerEventButton::Forward => "forward",
                PointerEventButton::Other => "none",
                _ => "none",
            }
        }

        let mut buttons_guard = self.pointer_buttons.lock();
        let mut buttons_mask = *buttons_guard;

        #[allow(unreachable_patterns)]
        match event.kind {
            PointerEventKind::Down
            | PointerEventKind::Up
            | PointerEventKind::Move
            | PointerEventKind::Cancel => {}
        }

        enum DevtoolsEvent<'a> {
            Mouse {
                kind: &'a str,
                button: &'a str,
                click_count: i32,
            },
            None,
        }

        #[allow(unreachable_patterns)]
        let devtools_event = match event.kind {
            PointerEventKind::Down => {
                let button = event.button;
                let mask = button_mask(button);
                let button_name = if mask != 0 {
                    buttons_mask |= mask;
                    button_name(button)
                } else {
                    "none"
                };
                DevtoolsEvent::Mouse {
                    kind: "mousePressed",
                    button: button_name,
                    click_count: 1,
                }
            }
            PointerEventKind::Up => {
                let button = event.button;
                let mask = button_mask(button);
                let button_name = if mask != 0 {
                    buttons_mask &= !mask;
                    button_name(button)
                } else {
                    "none"
                };
                DevtoolsEvent::Mouse {
                    kind: "mouseReleased",
                    button: button_name,
                    click_count: 1,
                }
            }
            PointerEventKind::Move => DevtoolsEvent::Mouse {
                kind: "mouseMoved",
                button: "none",
                click_count: 0,
            },
            PointerEventKind::Cancel => {
                buttons_mask = 0;
                DevtoolsEvent::Mouse {
                    kind: "mouseReleased",
                    button: "none",
                    click_count: 0,
                }
            }
            _ => DevtoolsEvent::None,
        };

        *buttons_guard = buttons_mask;
        drop(buttons_guard);

        match devtools_event {
            DevtoolsEvent::Mouse {
                kind,
                button,
                click_count,
            } => {
                // WebView2 might need coordinates relative to the WebView content area
                let (w, h) = self.composition.borrow().dimensions();

                let layout_metrics = self.get_viewport_metrics(w as f64, h as f64)?;
                let dom_metrics = match self.get_dom_viewport_metrics(w as f64, h as f64) {
                    Ok(metrics) => Some(metrics),
                    Err(err) => {
                        tracing::debug!(
                            "Falling back to layout metrics for pointer transform: {err:?}"
                        );
                        None
                    }
                };

                let fallback_dpr = if layout_metrics.device_pixel_ratio.is_finite()
                    && layout_metrics.device_pixel_ratio > 0.0
                {
                    layout_metrics.device_pixel_ratio
                } else {
                    1.0
                };

                let (mut device_pixel_ratio, mut scale_factor, css_width, css_height, dom_used) =
                    if let Some(dom) = dom_metrics {
                        (
                            if dom.device_pixel_ratio.is_finite() && dom.device_pixel_ratio > 0.0 {
                                dom.device_pixel_ratio
                            } else {
                                fallback_dpr
                            },
                            if dom.scale.is_finite() && dom.scale > 0.0 {
                                dom.scale
                            } else {
                                1.0
                            },
                            dom.width.max(0.0),
                            dom.height.max(0.0),
                            true,
                        )
                    } else {
                        (
                            fallback_dpr,
                            if layout_metrics.scale.is_finite() && layout_metrics.scale > 0.0 {
                                layout_metrics.scale
                            } else {
                                1.0
                            },
                            layout_metrics.visual_width.max(0.0),
                            layout_metrics.visual_height.max(0.0),
                            false,
                        )
                    };

                if !device_pixel_ratio.is_finite() || device_pixel_ratio <= f64::EPSILON {
                    device_pixel_ratio = 1.0;
                }
                if !scale_factor.is_finite() || scale_factor <= f64::EPSILON {
                    scale_factor = 1.0;
                }

                let mut css_x = x as f64 / device_pixel_ratio;
                let mut css_y = y as f64 / device_pixel_ratio;

                if (scale_factor - 1.0).abs() > f64::EPSILON {
                    css_x /= scale_factor;
                    css_y /= scale_factor;
                }

                let slint_width_css = (w as f64) / device_pixel_ratio / scale_factor;
                let slint_height_css = (h as f64) / device_pixel_ratio / scale_factor;

                let raw_width_ratio = if slint_width_css.abs() > f64::EPSILON {
                    css_width / slint_width_css
                } else {
                    1.0
                };
                let raw_height_ratio = if slint_height_css.abs() > f64::EPSILON {
                    css_height / slint_height_css
                } else {
                    1.0
                };

                css_x *= 1.0;
                css_y *= 1.0;

                let transformed_x = css_x;
                let transformed_y = css_y;

                let baseline_css_x = x as f64 / device_pixel_ratio / scale_factor;
                let baseline_css_y = y as f64 / device_pixel_ratio / scale_factor;

                let delta_x = transformed_x - baseline_css_x;
                let delta_y = transformed_y - baseline_css_y;

                info!("Coordinate mapping: slint=({:.1}, {:.1}) devtools=({:.1}, {:.1}) cssSize={:.1}x{:.1} ratios=({:.3}, {:.3}) dpr={:.3} scale={:.3} dom_used={} delta=({:.1}, {:.1})",
                x, y, transformed_x, transformed_y, css_width, css_height, raw_width_ratio, raw_height_ratio, device_pixel_ratio, scale_factor, dom_used, delta_x, delta_y);

                // Inject JS crosshair on first click if not already present
                let inject_js = r#"
                    (function() {
                        if (!window.debugCrosshairInjected) {
                            console.log('Injecting debug crosshair...');

                            const existing = document.getElementById('debug-crosshair');
                            if (existing) existing.remove();

                            const crosshair = document.createElement('div');
                            crosshair.id = 'debug-crosshair';
                            crosshair.style.position = 'fixed';
                            crosshair.style.pointerEvents = 'none';
                            crosshair.style.zIndex = '999999';
                            crosshair.style.display = 'none';
                            crosshair.style.width = '56px';
                            crosshair.style.height = '56px';
                            crosshair.style.marginLeft = '-28px';
                            crosshair.style.marginTop = '-28px';
                            crosshair.style.filter = 'drop-shadow(0 0 2px rgba(0,0,0,0.45))';

                            const svgNS = 'http://www.w3.org/2000/svg';
                            const svg = document.createElementNS(svgNS, 'svg');
                            svg.setAttribute('width', '56');
                            svg.setAttribute('height', '56');
                            svg.setAttribute('viewBox', '0 0 56 56');
                            svg.style.position = 'absolute';
                            svg.style.top = '0';
                            svg.style.left = '0';

                            const makeLine = (x1, y1, x2, y2) => {
                                const line = document.createElementNS(svgNS, 'line');
                                line.setAttribute('x1', x1);
                                line.setAttribute('y1', y1);
                                line.setAttribute('x2', x2);
                                line.setAttribute('y2', y2);
                                line.setAttribute('stroke', 'rgba(255, 0, 0, 0.95)');
                                line.setAttribute('stroke-width', '2');
                                line.setAttribute('stroke-linecap', 'round');
                                return line;
                            };

                            const horizontal = makeLine('0', '28', '56', '28');
                            const vertical = makeLine('28', '0', '28', '56');

                            const center = document.createElementNS(svgNS, 'circle');
                            center.setAttribute('cx', '28');
                            center.setAttribute('cy', '28');
                            center.setAttribute('r', '2.5');
                            center.setAttribute('fill', 'rgba(255, 0, 0, 0.95)');
                            center.setAttribute('stroke', 'rgba(0, 0, 0, 0.3)');
                            center.setAttribute('stroke-width', '1');

                            svg.appendChild(horizontal);
                            svg.appendChild(vertical);
                            svg.appendChild(center);

                            const info = document.createElement('div');
                            info.style.position = 'absolute';
                            info.style.top = '60px';
                            info.style.left = '50%';
                            info.style.padding = '2px 4px';
                            info.style.backgroundColor = 'rgba(0, 0, 0, 0.65)';
                            info.style.color = '#fff';
                            info.style.fontSize = '11px';
                            info.style.fontFamily = 'monospace';
                            info.style.borderRadius = '4px';
                            info.style.pointerEvents = 'none';
                            info.style.transform = 'translateX(-50%)';

                            crosshair.appendChild(svg);
                            crosshair.appendChild(info);
                            document.body.appendChild(crosshair);

                            window.debugCrosshairLast = null;
                            window.debugCrosshairUpdate = function(slintX, slintY, devX, devY, note) {
                                crosshair.style.left = devX + 'px';
                                crosshair.style.top = devY + 'px';
                                crosshair.style.display = 'block';
                                info.textContent = `DEV(${devX.toFixed(1)}, ${devY.toFixed(1)}) SL(${slintX.toFixed(1)}, ${slintY.toFixed(1)})` + (note ? ` ${note}` : '');
                                window.debugCrosshairLast = {
                                    slint: { x: slintX, y: slintY },
                                    devtools: { x: devX, y: devY },
                                    note: note ?? null,
                                };
                            };

                            window.debugCrosshairPos = function() {
                                return {
                                    last: window.debugCrosshairLast,
                                    viewport: { width: window.innerWidth, height: window.innerHeight },
                                    devicePixelRatio: window.devicePixelRatio
                                };
                            };

                            window.debugCrosshairInjected = true;
                            console.log('RED debug crosshair ready – call window.debugCrosshairUpdate(...)');
                        }
                    })();
                "#;

                if let Err(e) = self.execute_script(inject_js) {
                    tracing::warn!("Could not inject JS crosshair: {}", e);
                }

                let update_note = format!(
                    "ratio=({:.3},{:.3}) dpr={:.3} scale={:.3} delta=({:.1},{:.1}) dom={}",
                    raw_width_ratio,
                    raw_height_ratio,
                    device_pixel_ratio,
                    scale_factor,
                    delta_x,
                    delta_y,
                    dom_used
                );
                let update_js = format!(
                    "if (window.debugCrosshairUpdate) window.debugCrosshairUpdate({:.3}, {:.3}, {:.3}, {:.3}, '{}');",
                    x,
                    y,
                    transformed_x,
                    transformed_y,
                    update_note
                );
                if let Err(e) = self.execute_script(&update_js) {
                    tracing::debug!("Could not update JS crosshair: {}", e);
                }

                // Try to read JS crosshair position for comparison
                let js_log = format!(
                    "console.log('Slint click: ({:.1}, {:.1}) | JS crosshair:', window.debugCrosshairPos ? window.debugCrosshairPos() : 'not available');",
                    x,
                    y
                );
                if let Err(e) = self.execute_script(&js_log) {
                    tracing::debug!("Could not log coordinate comparison: {}", e);
                }

                let payload = json!({
                    "type": kind,
                    "x": transformed_x,
                    "y": transformed_y,
                    "button": button,
                    "buttons": buttons_mask,
                    "clickCount": click_count,
                });

                call_devtools_method(
                    &self.webview,
                    "Input.dispatchMouseEvent",
                    &payload.to_string(),
                )?;
            }
            DevtoolsEvent::None => {}
        }

        Ok(())
    }

    fn send_scroll(&self, delta_x: f32, delta_y: f32, x: f32, y: f32) -> Result<()> {
        let buttons_mask = *self.pointer_buttons.lock();
        let payload = json!({
            "type": "mouseWheel",
            "x": x as f64,
            "y": y as f64,
            "button": "none",
            "buttons": buttons_mask,
            "clickCount": 0,
            "deltaX": -(delta_x as f64),
            "deltaY": -(delta_y as f64),
        });

        call_devtools_method(
            &self.webview,
            "Input.dispatchMouseEvent",
            &payload.to_string(),
        )?;

        Ok(())
    }

    fn send_key(&self, event: &KeyEvent) -> Result<bool> {
        use slint::private_unstable_api::re_exports::{Key, KeyboardModifiers};

        let event_type_code = event.event_type as u8;
        let is_key_down = event_type_code == 0;
        let is_key_up = event_type_code == 1;

        if !(is_key_down || is_key_up) {
            // IME composition events are not forwarded for now.
            return Ok(false);
        }

        #[derive(Default)]
        struct KeyDetails {
            key: String,
            text: Option<String>,
            windows_vk: i32,
        }

        fn modifier_bits(mods: &KeyboardModifiers) -> i32 {
            let mut result = 0;
            if mods.shift {
                result |= 8;
            }
            if mods.control {
                result |= 4;
            }
            if mods.alt {
                result |= 1;
            }
            if mods.meta {
                result |= 2;
            }
            result
        }

        fn modifier_from_mask(bits: i32) -> Option<i32> {
            if (bits & 8) != 0 {
                Some(8)
            } else if (bits & 4) != 0 {
                Some(4)
            } else if (bits & 1) != 0 {
                Some(1)
            } else if (bits & 2) != 0 {
                Some(2)
            } else {
                None
            }
        }

        fn key_from_text(text: &str) -> KeyDetails {
            fn ascii_vk(ch: char) -> i32 {
                if ch.is_ascii_alphabetic() {
                    ch.to_ascii_uppercase() as i32
                } else {
                    ch as i32
                }
            }

            fn details(key: &str, text: Option<&str>, vk: i32) -> KeyDetails {
                KeyDetails {
                    key: key.to_string(),
                    text: text.map(|t| t.to_string()),
                    windows_vk: vk,
                }
            }

            if let Some(ch) = text.chars().next() {
                let backspace = char::from(Key::Backspace);
                let tab = char::from(Key::Tab);
                let enter = char::from(Key::Return);
                let escape = char::from(Key::Escape);
                let delete_key = char::from(Key::Delete);
                let insert_key = char::from(Key::Insert);
                let home_key = char::from(Key::Home);
                let end_key = char::from(Key::End);
                let page_up_key = char::from(Key::PageUp);
                let page_down_key = char::from(Key::PageDown);
                let left_arrow = char::from(Key::LeftArrow);
                let right_arrow = char::from(Key::RightArrow);
                let up_arrow = char::from(Key::UpArrow);
                let down_arrow = char::from(Key::DownArrow);
                let shift = char::from(Key::Shift);
                let shift_r = char::from(Key::ShiftR);
                let control = char::from(Key::Control);
                let control_r = char::from(Key::ControlR);
                let alt = char::from(Key::Alt);
                let alt_gr = char::from(Key::AltGr);
                let meta = char::from(Key::Meta);
                let meta_r = char::from(Key::MetaR);

                match ch {
                    c if c == backspace || c == '\u{0008}' => {
                        return details("Backspace", None, 0x08);
                    }
                    c if c == tab || c == '\t' => {
                        return details("Tab", Some("\t"), 0x09);
                    }
                    c if c == enter || c == '\r' || c == '\n' => {
                        return details("Enter", Some("\r"), 0x0D);
                    }
                    c if c == escape || c == '\u{001B}' => {
                        return details("Escape", None, 0x1B);
                    }
                    c if c == delete_key => {
                        return details("Delete", None, 0x2E);
                    }
                    c if c == insert_key => {
                        return details("Insert", None, 0x2D);
                    }
                    c if c == home_key => {
                        return details("Home", None, 0x24);
                    }
                    c if c == end_key => {
                        return details("End", None, 0x23);
                    }
                    c if c == page_up_key => {
                        return details("PageUp", None, 0x21);
                    }
                    c if c == page_down_key => {
                        return details("PageDown", None, 0x22);
                    }
                    c if c == left_arrow => {
                        return details("ArrowLeft", None, 0x25);
                    }
                    c if c == right_arrow => {
                        return details("ArrowRight", None, 0x27);
                    }
                    c if c == up_arrow => {
                        return details("ArrowUp", None, 0x26);
                    }
                    c if c == down_arrow => {
                        return details("ArrowDown", None, 0x28);
                    }
                    c if c == shift || c == shift_r => {
                        return details("Shift", None, 0x10);
                    }
                    c if c == control || c == control_r => {
                        return details("Control", None, 0x11);
                    }
                    c if c == alt || c == alt_gr => {
                        return details("Alt", None, 0x12);
                    }
                    c if c == meta || c == meta_r => {
                        return details("Meta", None, 0x5B);
                    }
                    _ => {}
                }

                if ch == ' ' {
                    return details(" ", Some(" "), 0x20);
                }

                if !text.is_empty() && !ch.is_ascii_control() {
                    return KeyDetails {
                        key: text.to_string(),
                        text: Some(text.to_string()),
                        windows_vk: ascii_vk(ch),
                    };
                }

                KeyDetails {
                    key: format!("U+{:04X}", ch as u32),
                    text: None,
                    windows_vk: ascii_vk(ch),
                }
            } else {
                KeyDetails::default()
            }
        }

        fn set_modifier_details(details: &mut KeyDetails, bit: i32) {
            match bit {
                8 => {
                    details.key = "Shift".into();
                    details.text = None;
                    details.windows_vk = 0x10;
                }
                4 => {
                    details.key = "Control".into();
                    details.text = None;
                    details.windows_vk = 0x11;
                }
                1 => {
                    details.key = "Alt".into();
                    details.text = None;
                    details.windows_vk = 0x12;
                }
                2 => {
                    details.key = "Meta".into();
                    details.text = None;
                    details.windows_vk = 0x5B;
                }
                _ => {}
            }
        }

        fn modifier_bit_for_key(key: &str) -> Option<i32> {
            match key {
                "Shift" | "ShiftLeft" | "ShiftRight" => Some(8),
                "Control" | "ControlLeft" | "ControlRight" => Some(4),
                "Alt" | "AltLeft" | "AltRight" => Some(1),
                "Meta" | "MetaLeft" | "MetaRight" => Some(2),
                _ => None,
            }
        }

        fn code_for_key(key: &str, text: &str, windows_vk: i32) -> Option<String> {
            if key.len() == 1 {
                let mut chars = key.chars();
                if let Some(ch) = chars.next() {
                    if ch.is_ascii_alphabetic() {
                        return Some(format!("Key{}", ch.to_ascii_uppercase()));
                    }
                    if ch.is_ascii_digit() {
                        return Some(format!("Digit{}", ch));
                    }
                }
            }

            match key {
                "Backspace" => Some("Backspace".into()),
                "Tab" => Some("Tab".into()),
                "Enter" => Some("Enter".into()),
                "Escape" => Some("Escape".into()),
                "Space" | " " => Some("Space".into()),
                "ArrowLeft" => Some("ArrowLeft".into()),
                "ArrowRight" => Some("ArrowRight".into()),
                "ArrowUp" => Some("ArrowUp".into()),
                "ArrowDown" => Some("ArrowDown".into()),
                "Shift" => Some("ShiftLeft".into()),
                "Control" => Some("ControlLeft".into()),
                "Alt" => Some("AltLeft".into()),
                "Meta" => Some("MetaLeft".into()),
                "PageUp" => Some("PageUp".into()),
                "PageDown" => Some("PageDown".into()),
                "Home" => Some("Home".into()),
                "End" => Some("End".into()),
                "Insert" => Some("Insert".into()),
                "Delete" => Some("Delete".into()),
                _ => {
                    if !text.is_empty() && text.len() == 1 {
                        let ch = text.chars().next().unwrap();
                        if ch.is_ascii_alphanumeric() {
                            if ch.is_ascii_alphabetic() {
                                return Some(format!("Key{}", ch.to_ascii_uppercase()));
                            }
                            if ch.is_ascii_digit() {
                                return Some(format!("Digit{}", ch));
                            }
                        }
                    }
                    match windows_vk {
                        0x70..=0x7B => Some(format!("F{}", windows_vk - 0x6F)),
                        _ => None,
                    }
                }
            }
        }

        let mut details = key_from_text(event.text.as_str());

        if details.key.is_empty() {
            details.key = "Unidentified".into();
        }

        let mut modifier_guard = self.key_modifiers.lock();
        let previous_mask = *modifier_guard;
        let event_mask = modifier_bits(&event.modifiers);
        let mut key_bit = modifier_bit_for_key(details.key.as_str());

        if key_bit.is_none() {
            let candidate_bits = if is_key_down {
                event_mask & !previous_mask
            } else if is_key_up {
                previous_mask & !event_mask
            } else {
                0
            };
            key_bit = modifier_from_mask(candidate_bits);
        }

        if let Some(bit) = key_bit {
            if details.key == "Unidentified" || details.windows_vk == 0 {
                set_modifier_details(&mut details, bit);
            }
        }

        if let Some(bit) = key_bit {
            if is_key_down {
                *modifier_guard |= bit;
            } else if is_key_up {
                *modifier_guard &= !bit;
            }
        }

        if is_key_up {
            *modifier_guard &= !(8 | 4 | 1 | 2);
        }
        *modifier_guard |= event_mask;

        let modifiers = *modifier_guard;
        let allow_char_event = (modifiers & (4 | 1 | 2)) == 0;
        let is_system_key = (modifiers & 1) != 0;
        drop(modifier_guard);

        let mut base_payload = json!({
            "key": details.key,
            "windowsVirtualKeyCode": details.windows_vk,
            "nativeVirtualKeyCode": details.windows_vk,
            "keyCode": details.windows_vk,
            "modifiers": modifiers,
            "autoRepeat": event.repeat,
            "isKeypad": false,
            "location": 0,
        });

        if let Some(code) = code_for_key(
            details.key.as_str(),
            event.text.as_str(),
            details.windows_vk,
        ) {
            base_payload["code"] = json!(code);
        }

        if is_key_down {
            let mut raw_down = base_payload.clone();
            raw_down["type"] = json!("rawKeyDown");
            if let Some(text) = details.text.as_ref() {
                if !text.is_empty() {
                    raw_down["text"] = json!(text);
                    raw_down["unmodifiedText"] = json!(text);
                }
            }
            if is_system_key {
                raw_down["isSystemKey"] = json!(true);
            }
            call_devtools_method(
                &self.webview,
                "Input.dispatchKeyEvent",
                &raw_down.to_string(),
            )?;

            let mut key_down = base_payload.clone();
            key_down["type"] = json!("keyDown");
            if let Some(text) = details.text.as_ref() {
                if !text.is_empty() {
                    key_down["text"] = json!(text);
                    key_down["unmodifiedText"] = json!(text);
                }
            }
            if is_system_key {
                key_down["isSystemKey"] = json!(true);
            }
            call_devtools_method(
                &self.webview,
                "Input.dispatchKeyEvent",
                &key_down.to_string(),
            )?;

            if allow_char_event {
                if let Some(text) = details.text.as_ref() {
                    if !text.is_empty() {
                        let mut char_event = base_payload.clone();
                        char_event["type"] = json!("char");
                        char_event["text"] = json!(text);
                        char_event["unmodifiedText"] = json!(text);
                        char_event["key"] = json!(text);
                        call_devtools_method(
                            &self.webview,
                            "Input.dispatchKeyEvent",
                            &char_event.to_string(),
                        )?;
                    }
                }
            }
        }

        if is_key_up {
            let mut key_up = base_payload.clone();
            key_up["type"] = json!("keyUp");
            if is_system_key {
                key_up["isSystemKey"] = json!(true);
            }
            call_devtools_method(&self.webview, "Input.dispatchKeyEvent", &key_up.to_string())?;
        }

        Ok(true)
    }
}

#[cfg(target_os = "windows")]
#[cfg(target_os = "windows")]
fn call_devtools_method(webview: &ICoreWebView2, method: &str, parameters: &str) -> Result<()> {
    unsafe {
        let method_h = HSTRING::from(method);
        let params_h = HSTRING::from(parameters);
        let label = method.to_string();
        let handler = CallDevToolsProtocolMethodCompletedHandler::create(Box::new(
            move |error_code, _result| {
                if error_code.is_err() {
                    warn!("DevTools method {} reported error: {:?}", label, error_code);
                }
                Ok(())
            },
        ));
        webview.CallDevToolsProtocolMethod(&method_h, &params_h, &handler)?;
    }

    Ok(())
}

#[cfg(target_os = "windows")]
fn call_devtools_method_with_result(
    webview: &ICoreWebView2,
    method: &str,
    parameters: &str,
) -> Result<String> {
    let (tx, rx) = mpsc::channel::<Result<String>>();

    unsafe {
        let method_h = HSTRING::from(method);
        let params_h = HSTRING::from(parameters);
        let label = method.to_string();
        let handler = CallDevToolsProtocolMethodCompletedHandler::create(Box::new(
            move |error_code, result| {
                let outcome: Result<String> = if let Err(err) = error_code {
                    Err(anyhow!(
                        "DevTools method {} reported error: {:?}",
                        label,
                        err
                    ))
                } else {
                    Ok(result)
                };
                let _ = tx.send(outcome);
                Ok(())
            },
        ));
        webview.CallDevToolsProtocolMethod(&method_h, &params_h, &handler)?;
    }

    match wait_with_pump(rx) {
        Ok(outcome) => outcome,
        Err(e) => Err(anyhow!(
            "wait_with_pump failed while awaiting DevTools response: {e:?}"
        )),
    }
}

#[cfg(target_os = "windows")]
impl Drop for WindowsWebViewState {
    fn drop(&mut self) {
        unsafe {
            let _ = self.controller.Close();
            if !self.hwnd.is_invalid() {
                let _ = DestroyWindow(self.hwnd);
            }
        }
    }
}

#[cfg(target_os = "windows")]
fn create_environment() -> Result<ICoreWebView2Environment3> {
    let (tx, rx) = mpsc::channel::<Result<ICoreWebView2Environment3, Error>>();
    let options = CoreWebView2EnvironmentOptions::default();

    unsafe {
        let handler = CreateCoreWebView2EnvironmentCompletedHandler::create(Box::new(
            move |result, environment| {
                let outcome = match result {
                    Ok(()) => {
                        if let Some(env) = environment {
                            env.cast()
                        } else {
                            Err(Error::from(E_FAIL))
                        }
                    }
                    Err(err) => Err(err),
                };

                let _ = tx.send(outcome);
                Ok(())
            },
        ));

        CreateCoreWebView2EnvironmentWithOptions(
            PCWSTR::null(),
            PCWSTR::null(),
            &ICoreWebView2EnvironmentOptions::from(options),
            &handler,
        )?;
    }

    match wait_with_pump(rx) {
        Ok(result) => result.map_err(|e| anyhow!("WebView2 environment creation failed: {e:?}")),
        Err(e) => Err(anyhow!(
            "wait_with_pump failed while creating environment: {e:?}"
        )),
    }
}

#[cfg(target_os = "windows")]
fn create_controller(
    environment: &ICoreWebView2Environment3,
    hwnd: HWND,
) -> Result<(ICoreWebView2Controller, ICoreWebView2CompositionController)> {
    let (tx, rx) = mpsc::channel::<
        Result<(ICoreWebView2Controller, ICoreWebView2CompositionController), Error>,
    >();

    unsafe {
        let handler = CreateCoreWebView2CompositionControllerCompletedHandler::create(Box::new(
            move |result, controller| {
                let outcome = match result {
                    Ok(()) => {
                        if let Some(ctrl) = controller {
                            match ctrl.cast() {
                                Ok(base) => Ok((base, ctrl.clone())),
                                Err(err) => Err(err),
                            }
                        } else {
                            Err(Error::from(E_FAIL))
                        }
                    }
                    Err(err) => Err(err),
                };

                let _ = tx.send(outcome);
                Ok(())
            },
        ));

        environment.CreateCoreWebView2CompositionController(hwnd, &handler)?;
    }

    match wait_with_pump(rx) {
        Ok(result) => result.map_err(|e| anyhow!("Failed to create WebView2 controller: {e:?}")),
        Err(e) => Err(anyhow!(
            "wait_with_pump failed while creating WebView2 controller: {e:?}"
        )),
    }
}

#[cfg(target_os = "windows")]
fn create_hidden_window(width: u32, height: u32) -> Result<HWND> {
    unsafe extern "system" fn window_proc(
        hwnd: HWND,
        msg: u32,
        wparam: WPARAM,
        lparam: LPARAM,
    ) -> LRESULT {
        unsafe { DefWindowProcW(hwnd, msg, wparam, lparam) }
    }

    unsafe {
        let hinstance = GetModuleHandleW(PCWSTR::null())?;
        let class_name = HSTRING::from("SlintWebView2HiddenWindow");
        let wnd_class = WNDCLASSW {
            style: CS_HREDRAW | CS_VREDRAW,
            lpfnWndProc: Some(window_proc),
            hInstance: hinstance.into(),
            lpszClassName: PCWSTR(class_name.as_ptr()),
            ..Default::default()
        };

        // It's fine if the class was already registered.
        RegisterClassW(&wnd_class);

        let hwnd = CreateWindowExW(
            WINDOW_EX_STYLE::default(),
            PCWSTR(class_name.as_ptr()),
            PCWSTR(class_name.as_ptr()),
            WS_OVERLAPPEDWINDOW,
            CW_USEDEFAULT,
            CW_USEDEFAULT,
            width as i32,
            height as i32,
            HWND::default(),
            None,
            hinstance,
            None,
        )?;

        let _ = ShowWindow(hwnd, SW_HIDE);
        Ok(hwnd)
    }
}
