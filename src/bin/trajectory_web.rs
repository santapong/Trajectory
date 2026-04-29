//! Trajectory web server.
//!
//! One Rust binary serves the SPA + an HTTP API. Bundled with `--features web`.
//!
//!   trajectory-web --tool tests/fixtures/tool_ball6.json \
//!                  --frame tests/fixtures/workpiece_identity.json \
//!                  --addr 127.0.0.1:8080
//!
//! Endpoints:
//!   GET  /                    - embedded SPA (HTML + JS + three.js CDN)
//!   GET  /api/health          - {"status":"ok","version":"..."}
//!   GET  /api/defaults        - default ReliefRequest as JSON
//!   POST /api/relief          - multipart: image=<png>, request=<json> -> JSON
//!                                {rapid, toolpath, stats}

use std::net::SocketAddr;
use std::path::PathBuf;
use std::sync::Arc;

use axum::extract::{Multipart, State};
use axum::http::{header, StatusCode};
use axum::response::{IntoResponse, Response};
use axum::routing::{get, post};
use axum::{Json, Router};
use clap::Parser;
use serde::Serialize;
use tower_http::cors::CorsLayer;
use tower_http::trace::TraceLayer;
use tracing::{error, info};
use tracing_subscriber::EnvFilter;

use trajectory::frame::WorkpieceFrame;
use trajectory::relief_job::{run_from_bytes, JobError, JobInputs, ReliefRequest, ReliefStats};
use trajectory::tool::Tool;

const INDEX_HTML: &str = include_str!("../../frontend/index.html");

#[derive(Parser, Debug)]
#[command(name = "trajectory-web", version)]
struct Args {
    #[arg(long)]
    tool: PathBuf,
    #[arg(long)]
    frame: PathBuf,
    #[arg(long, default_value = "127.0.0.1:8080")]
    addr: SocketAddr,
}

#[derive(Clone)]
struct AppState {
    tool: Arc<Tool>,
    frame: Arc<WorkpieceFrame>,
    tool_path: Arc<String>,
    frame_path: Arc<String>,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let filter = EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info,tower_http=warn"));
    tracing_subscriber::fmt()
        .with_env_filter(filter)
        .with_target(false)
        .init();

    let args = Args::parse();
    let tool = Tool::load_json(&args.tool)?;
    let frame = WorkpieceFrame::load_json(&args.frame)?;
    info!(addr = %args.addr, tool = %tool.name, "trajectory-web starting");

    let state = AppState {
        tool: Arc::new(tool),
        frame: Arc::new(frame),
        tool_path: Arc::new(args.tool.display().to_string()),
        frame_path: Arc::new(args.frame.display().to_string()),
    };

    let app = Router::new()
        .route("/", get(index))
        .route("/api/health", get(health))
        .route("/api/defaults", get(defaults))
        .route("/api/relief", post(relief))
        .with_state(state)
        .layer(CorsLayer::permissive())
        .layer(TraceLayer::new_for_http());

    let listener = tokio::net::TcpListener::bind(&args.addr).await?;
    info!("listening on http://{}", args.addr);
    axum::serve(listener, app).await?;
    Ok(())
}

async fn index() -> impl IntoResponse {
    Response::builder()
        .status(StatusCode::OK)
        .header(header::CONTENT_TYPE, "text/html; charset=utf-8")
        .body(INDEX_HTML.to_string())
        .unwrap()
}

#[derive(Serialize)]
struct Health {
    status: &'static str,
    version: &'static str,
}

async fn health() -> Json<Health> {
    Json(Health {
        status: "ok",
        version: env!("CARGO_PKG_VERSION"),
    })
}

async fn defaults() -> Json<ReliefRequest> {
    Json(ReliefRequest::default())
}

#[derive(Serialize)]
struct ReliefResponse {
    rapid: String,
    /// Already-encoded toolpath JSON (the frontend just re-parses it).
    toolpath: serde_json::Value,
    stats: ReliefStats,
}

#[derive(Serialize)]
struct ApiError {
    error: String,
}

impl IntoResponse for ApiError {
    fn into_response(self) -> Response {
        (StatusCode::BAD_REQUEST, Json(self)).into_response()
    }
}

async fn relief(
    State(state): State<AppState>,
    mut multipart: Multipart,
) -> Result<Json<ReliefResponse>, ApiError> {
    let mut png_bytes: Option<Vec<u8>> = None;
    let mut png_label: String = "<upload>".into();
    let mut request: Option<ReliefRequest> = None;

    while let Some(field) = multipart.next_field().await.map_err(|e| ApiError {
        error: format!("multipart: {e}"),
    })? {
        let name = field.name().unwrap_or("").to_string();
        match name.as_str() {
            "image" => {
                if let Some(fname) = field.file_name() {
                    png_label = fname.to_string();
                }
                let bytes = field.bytes().await.map_err(|e| ApiError {
                    error: format!("read image: {e}"),
                })?;
                png_bytes = Some(bytes.to_vec());
            }
            "request" => {
                let bytes = field.bytes().await.map_err(|e| ApiError {
                    error: format!("read request: {e}"),
                })?;
                let req: ReliefRequest =
                    serde_json::from_slice(&bytes).map_err(|e| ApiError {
                        error: format!("parse request json: {e}"),
                    })?;
                request = Some(req);
            }
            _ => {}
        }
    }

    let png = png_bytes.ok_or(ApiError {
        error: "missing 'image' part".into(),
    })?;
    let req = request.unwrap_or_default();

    let out = tokio::task::spawn_blocking({
        let tool = state.tool.clone();
        let frame = state.frame.clone();
        let tool_path = state.tool_path.clone();
        let frame_path = state.frame_path.clone();
        let png_label = png_label.clone();
        move || {
            let inputs = JobInputs::new(&tool, &frame)
                .with_tool_path(tool_path.as_str())
                .with_frame_path(frame_path.as_str());
            run_from_bytes(&png, &png_label, &req, &inputs)
        }
    })
    .await
    .map_err(|e| ApiError {
        error: format!("internal: {e}"),
    })?;

    let out = match out {
        Ok(o) => o,
        Err(e) => {
            // Validation errors are user-actionable; log at info, return 400.
            let msg = match &e {
                JobError::InvalidParams(m) => m.clone(),
                JobError::InvalidRequest(m) => m.clone(),
                JobError::Surface(s) => format!("surface: {s}"),
                JobError::Post(p) => format!("post: {p}"),
                JobError::Io(io) => format!("io: {io}"),
            };
            error!(error = %msg, "relief job failed");
            return Err(ApiError { error: msg });
        }
    };

    let toolpath: serde_json::Value =
        serde_json::from_str(&out.toolpath_json).unwrap_or(serde_json::json!({}));
    Ok(Json(ReliefResponse {
        rapid: out.rapid,
        toolpath,
        stats: out.stats,
    }))
}
