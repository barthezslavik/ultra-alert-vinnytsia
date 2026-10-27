//! Ultra Alert: jet-Shahed tracking and impact ETA for Vinnytsia from Telegram channels.
//! The menu-bar app is `src/main.rs`; this library also holds the data pipeline that keeps
//! the model trained (`ultra-alert daily`).

pub mod archive;
pub mod gazetteer;
pub mod lookup;
pub mod pipeline;
pub mod predict;
pub mod telegram;
