pub mod files;
pub mod play;
pub mod stream;

use std::{net::IpAddr, path::PathBuf, sync::Arc};

use anyhow::Result as AnyResult;
use clap::Args;
use floppy_music_middle::sequencer::MidiEngine;
use rocket::{
    form::Form,
    fs::{FileServer, NamedFile},
    response::Redirect,
    serde::json::json,
    State,
};
use rocket_auth::{Auth, Login, Signup, User, Users};
use rocket_dyn_templates::Template;
use tokio::{
    io::{AsyncReadExt, AsyncWriteExt},
    select,
    sync::{broadcast, mpsc, oneshot, RwLock},
};
use tokio_serial::SerialStream;

use crate::web::{
    files::*,
    play::*,
    stream::{log_stream, panel_stream, PanelEventType},
};
use crate::{CommonArgs, ProgramSignal};

#[derive(Debug, Args)]
pub struct WebArgs {
    #[clap(short, long = "bind", default_value = "127.0.0.1")]
    bind_addr: IpAddr,

    #[clap(short, long, default_value_t = 1337)]
    port: u16,

    #[clap(long, default_value = concat!(env!("CARGO_MANIFEST_DIR"), "/midi"))]
    midi_files: PathBuf,

    #[clap(long, default_value = "users.db")]
    users_db: PathBuf,

    #[clap(long, default_value = concat!(env!("CARGO_MANIFEST_DIR"), "/frontend/dist"))]
    static_files: PathBuf,

    #[clap(long, default_value = "/dev/ttyACM0")]
    serial_port: String,
}

#[post("/login", data = "<form>")]
async fn post_login(form: Form<Login>, auth: Auth<'_>) -> Result<Redirect, rocket_auth::Error> {
    let result = auth.login(&form).await;
    println!("login attempt: {:?}", result);

    result?;

    //Ok(Json(auth.get_user().await.unwrap()))
    Ok(Redirect::to("/"))
}

#[derive(Responder)]
enum LoginResponse {
    LoggedIn(Redirect),
    NotLoggedIn(NamedFile),
}

#[get("/login")]
async fn get_login(args: &State<WebArgs>, user: Option<User>) -> LoginResponse {
    if user.is_some() {
        return LoginResponse::LoggedIn(Redirect::to("/"));
    }

    LoginResponse::NotLoggedIn(
        NamedFile::open(args.inner().static_files.join("index.html"))
            .await
            .unwrap(),
    )
}

#[get("/logout")]
fn api_logout(auth: Auth<'_>) -> Result<(), rocket_auth::Error> {
    auth.logout()?;

    Ok(())
}

#[get("/logout")]
fn logout(auth: Auth<'_>) -> Result<Redirect, rocket_auth::Error> {
    auth.logout()?;

    Ok(Redirect::to("/login"))
}

#[derive(Responder)]
enum IndexResponse {
    NotLoggedIn(Redirect),
    LoggedIn(NamedFile),
}

#[get("/")]
async fn index(args: &State<WebArgs>, user: Option<User>) -> IndexResponse {
    if user.is_none() {
        return IndexResponse::NotLoggedIn(Redirect::to("/login"));
    };

    IndexResponse::LoggedIn(
        NamedFile::open(args.inner().static_files.join("index.html"))
            .await
            .unwrap(),
    )
}

pub async fn start(
    args: WebArgs,
    common: CommonArgs,
    signal_rx: oneshot::Receiver<ProgramSignal>,
) -> AnyResult<()> {
    let users = Users::open_rusqlite(&args.users_db)?;

    let _ = users
        .create_user("josephcahunt@jcah.uk", "securedPassword", true)
        .await;

    let _ = users.create_user("1@jcah.uk", "1234", false).await;

    let _ = users.create_user("2@jcah.uk", "2345", false).await;

    let mut files = FileMap::new("midi-meta.toml", &args.midi_files);
    files.refresh_from_dir().await?;

    let (pi_tx, pi_rx) = {
        let (out_tx, mut out_rx) = mpsc::channel::<Vec<u8>>(100);
        let (in_tx, in_rx) = broadcast::channel(100);

        let serial_port = SerialStream::open(&tokio_serial::new(&args.serial_port, 115_200));

        tokio::spawn(async move {
            if let Ok(mut serial_port) = serial_port {
                loop {
                    let mut buf = [0u8; 256];
                    tokio::select! {
                        val = serial_port.read(&mut buf[..]) => {
                            if let Ok(len) = val {
                                let _ = in_tx.send(String::from_utf8(buf[..len].to_vec()).unwrap());
                            }
                        }

                        Some(msg) = out_rx.recv() => {
                            let _ = serial_port.write(msg.as_slice()).await;
                        }

                        else => {}
                    }
                }
            } else {
                loop {}
            }
        });

        (out_tx, in_rx)
    };

    let (event_tx, event_rx) = broadcast::channel::<PanelEventType>(10);

    let playing: Option<Playing> = None;
    let playing = Arc::new(RwLock::new(playing));

    let engine = MidiEngine::new();
    {
        let mut state = engine.watch_state();
        let tx = event_tx.clone();
        tokio::spawn(async move {
            while state.changed().await.is_ok() {
                tx.send(PanelEventType::StatusChange);
            }
        });

        let mut state = engine.watch_state();
        let tx = event_tx.clone();
        let playing = playing.clone();
        tokio::spawn(async move {
            while state.changed().await.is_ok() {
                if state.borrow().is_stopped() {
                    let mut playing = playing.write().await;

                    *playing = None;

                    tx.send(PanelEventType::PlayingFileChanged);
                }
            }
        });
    }

    {
        let mut pi_rx = pi_rx.resubscribe();
        tokio::spawn(async move {
            loop {
                if let Ok(msg) = pi_rx.recv().await {
                    print!("{}", msg);
                }
            }
        });
    }

    let engine = RwLock::new(engine);
    let files = Arc::new(RwLock::new(files));

    let rocket = rocket::build()
        .mount("/", routes![index, get_login, logout])
        .mount("/", FileServer::from(&args.static_files))
        .mount(
            "/api/v1",
            routes![
                post_login,
                api_logout,
                get_files,
                get_file,
                get_play,
                get_playing,
                get_status,
                get_pause,
                get_resume,
                get_stop,
                post_play_msg,
            ],
        )
        .mount("/api/v1/events", routes![panel_stream, log_stream])
        .manage(files)
        .manage(args)
        .manage(users)
        .manage(engine)
        .manage(pi_tx)
        .manage(event_tx)
        .manage(event_rx)
        .manage(pi_rx)
        .manage(playing)
        .attach(Template::fairing())
        .launch();

    select! {
        result = rocket => { result?; },
        _ = signal_rx => {},
    }

    Ok(())
}
