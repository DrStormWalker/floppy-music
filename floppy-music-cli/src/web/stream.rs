use rocket::{
    response::stream::{Event, EventStream},
    serde::Serialize,
    State,
};
use rocket_auth::User;
use tokio::sync::broadcast;
use tokio_stream::{wrappers::BroadcastStream, StreamExt};

#[derive(Copy, Clone)]
pub enum PanelEventType {
    FilesUpdated,
    PlayingFileChanged,
    StatusChange,
}
impl ToString for PanelEventType {
    fn to_string(&self) -> String {
        match self {
            Self::FilesUpdated => "files",
            Self::PlayingFileChanged => "playing",
            Self::StatusChange => "status",
        }
        .to_string()
    }
}

#[derive(Clone, Serialize)]
pub struct PanelEvent {
    pub path: String,
}

#[get("/panel")]
pub fn panel_stream(rx: &State<broadcast::Receiver<PanelEventType>>, user: User) -> EventStream![] {
    EventStream::from(BroadcastStream::new(rx.resubscribe()).filter_map(|e| {
        Some(Event::json(&PanelEvent {
            path: e.ok()?.to_string(),
        }))
    }))
}

#[get("/logs")]
pub fn log_stream(rx: &State<broadcast::Receiver<String>>, user: User) -> EventStream![] {
    EventStream::from(
        BroadcastStream::new(rx.resubscribe()).filter_map(|e| Some(Event::json(&e.ok()?))),
    )
}
