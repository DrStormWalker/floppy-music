use std::{net::IpAddr, path::PathBuf};

use clap::Args;
use rocket::{form::Form, fs::FileServer, response::Redirect, serde::json::json};
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
    Ok(Redirect::to("/"))
}

#[get("/login")]
fn get_login() -> Template {
    Template::render("login", json!({}))
}

#[get("/logout")]
fn logout(auth: Auth<'_>) -> Result<Template, rocket_auth::Error> {
    auth.logout()?;
    Ok(Template::render("logout", json!({})))
}

#[get("/")]
async fn index(user: Option<User>) -> Template {
    Template::render("index", json!({ "user": user }))
}

pub async fn start(
    args: WebArgs,
    common: CommonArgs,
    signal_rx: oneshot::Receiver<ProgramSignal>,
) -> Result<(), Box<dyn std::error::Error>> {
    let users = Users::open_rusqlite(args.users_db)?;

    let _ = users
        .create_user("josephcahunt@jcah.uk", "1234", true)
        .await;

    let rocket = rocket::build()
        .mount("/", FileServer::from(args.static_files))
        .mount("/api", routes![index, get_login, post_login, logout])
        .manage(users)
        .attach(Template::fairing())
        .launch();

    select! {
        result = rocket => { result?; },
        _ = signal_rx => {},
    }

    Ok(())
}
