use std::sync::Arc;

use floppy_music_middle::sequencer::{EngineState, MidiEngine};
use midly::live::LiveEvent;
use rocket::{
    form::Form,
    response::status::{Conflict, NotFound},
    serde::{json::Json, Serialize},
    State,
};
use rocket_auth::User;
use tokio::{
    fs,
    sync::{broadcast, mpsc, RwLock},
};

use super::{files::FileMap, stream::PanelEventType};

#[derive(Clone, Serialize)]
pub struct Playing {
    pub current: String,
}

#[get("/playing")]
pub async fn get_playing(
    user: User,
    current: &State<Arc<RwLock<Option<Playing>>>>,
) -> Json<Option<Playing>> {
    Json(current.read().await.clone())
}

#[derive(Clone, Serialize)]
pub struct Status {
    status: EngineState,
}

#[get("/status")]
pub async fn get_status(user: User, engine: &State<RwLock<MidiEngine>>) -> Json<Status> {
    Json(Status {
        status: engine.read().await.get_state(),
    })
}

#[get("/resume")]
pub async fn get_resume(
    user: User,
    engine: &State<RwLock<MidiEngine>>,
) -> Result<(), Conflict<String>> {
    engine
        .read()
        .await
        .resume()
        .map_err(|_| Conflict(Some("Engine is not paused".to_string())))
}

#[get("/pause")]
pub async fn get_pause(
    user: User,
    engine: &State<RwLock<MidiEngine>>,
) -> Result<(), Conflict<String>> {
    engine
        .read()
        .await
        .pause()
        .map_err(|_| Conflict(Some("Engine is not running".to_string())))
}

#[get("/stop")]
pub async fn get_stop(
    user: User,
    engine: &State<RwLock<MidiEngine>>,
    events_tx: &State<broadcast::Sender<PanelEventType>>,
) {
    engine.write().await.stop().await;
}

#[get("/play/<file>")]
pub async fn get_play(
    user: User,
    engine: &State<RwLock<MidiEngine>>,
    pi_tx: &State<mpsc::Sender<Vec<u8>>>,
    playing: &State<Arc<RwLock<Option<Playing>>>>,
    file_map: &State<Arc<RwLock<FileMap>>>,
    events_tx: &State<broadcast::Sender<PanelEventType>>,
    file: &str,
) -> Result<(), NotFound<String>> {
    let bytes = fs::read(
        file_map
            .read()
            .await
            .map
            .read()
            .await
            .get(file)
            .ok_or(NotFound(format!("File: {} could not be found", file)))?
            .path
            .clone(),
    )
    .await
    .unwrap();

    let mut engine = engine.write().await;
    engine.play_to(&bytes, (*pi_tx).clone()).await;
    {
        let mut playing = playing.write().await;
        if let Some(playing) = playing.as_mut() {
            playing.current = file.to_string();
        } else {
            *playing = Some(Playing {
                current: file.to_string(),
            })
        }
    }
    events_tx.send(PanelEventType::PlayingFileChanged);

    Ok(())
}

#[post("/play/msg", data = "<msg>")]
pub async fn post_play_msg(
    user: User,
    pi_tx: &State<mpsc::Sender<Vec<u8>>>,
    msg: Vec<u8>,
) -> Result<(), ()> {
    println!("Buffer: {:?}", msg);
    let event = LiveEvent::parse(&msg).map_err(|e| ())?;

    println!("Event: {:?}", event);

    pi_tx.inner().send(msg.clone()).await.map_err(|e| ())
}
