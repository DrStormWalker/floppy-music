use std::{net::IpAddr, path::PathBuf};

use clap::Args;
use rocket::{
    form::Form,
    fs::{FileServer, NamedFile},
    response::Redirect,
    serde::json::json,
    State,
};
use rocket_auth::{Auth, Login, Signup, User, Users};
use rocket_dyn_templates::Template;
use tokio::{select, sync::oneshot};

use crate::{CommonArgs, ProgramSignal};

#[derive(Debug, Args)]
pub struct WebArgs {
    #[clap(short, long = "bind", default_value = "127.0.0.1")]
    bind_addr: IpAddr,

    #[clap(short, long, default_value_t = 1337)]
    port: u16,

    #[clap(long, default_value = "")]
    midi_files: PathBuf,

    #[clap(long, default_value = "users.db")]
    users_db: PathBuf,

    #[clap(long, default_value = concat!(env!("CARGO_MANIFEST_DIR"), "/frontend/dist"))]
    static_files: PathBuf,
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
) -> Result<(), Box<dyn std::error::Error>> {
    let users = Users::open_rusqlite(&args.users_db)?;

    let _ = users
        .create_user("josephcahunt@jcah.uk", "1234", true)
        .await;

    let rocket = rocket::build()
        .mount("/", routes![index, get_login, logout])
        .mount("/", FileServer::from(&args.static_files))
        .mount("/api/v1", routes![post_login, api_logout])
        .manage(args)
        .manage(users)
        .attach(Template::fairing())
        .launch();

    select! {
        result = rocket => { result?; },
        _ = signal_rx => {},
    }

    Ok(())
}
