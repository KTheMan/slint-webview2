#![warn(missing_docs)]

//! Slint-friendly WebView2 integration helpers.
//!
//! This crate contains legacy/experimental implementations that are kept in a
//! separate repository boundary.

#[cfg(feature = "slint")]
pub mod slint_webview2;

#[cfg(feature = "renderer")]
pub mod webview2_renderer;

#[cfg(feature = "slint")]
pub use slint_webview2::*;
