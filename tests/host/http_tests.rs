//! Actual picoserve router exercised over loopback TCP; no physical GPIO or flash.
use crate::{heater_control::SafetyLimits, http_api::{self, AppState, Shared}, persist::Settings};
use std::{cell::RefCell, rc::Rc, time::Duration};
use tokio::{io::{AsyncReadExt, AsyncWriteExt}, net::{TcpListener, TcpStream}};

fn app() -> Shared {
    Rc::new(RefCell::new(AppState::new(Some(SafetyLimits { max_temperature_mc: 35_000, heater_watts: 100 }), Settings::default())))
}

async fn request(shared: Shared, method: &str, path: &str, body: &str, content_type: &str) -> (u16, String) {
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    let server = async {
        let (socket, _) = listener.accept().await.unwrap();
        let router = http_api::router(shared);
        let config = picoserve::Config::new(picoserve::Timeouts {
            start_read_request: picoserve::time::Duration::from_secs(2),
            persistent_start_read_request: picoserve::time::Duration::from_secs(2),
            read_request: picoserve::time::Duration::from_secs(2),
            write: picoserve::time::Duration::from_secs(2),
        }).close_connection_after_response();
        picoserve::Server::new_tokio(&router, &config, &mut [0u8; 2048]).serve(socket).await.unwrap();
    };
    let client = async {
        let mut socket = TcpStream::connect(addr).await.unwrap();
        let raw = format!("{method} {path} HTTP/1.1\r\nHost: localhost\r\nConnection: close\r\nContent-Type: {content_type}\r\nContent-Length: {}\r\n\r\n{body}", body.len());
        socket.write_all(raw.as_bytes()).await.unwrap();
        socket.shutdown().await.unwrap();
        let mut response = Vec::new();
        socket.read_to_end(&mut response).await.unwrap();
        let raw = String::from_utf8(response).unwrap();
        let (headers, body) = raw.split_once("\r\n\r\n").expect("HTTP framing");
        let code = headers.split_whitespace().nth(1).unwrap().parse().unwrap();
        (code, body.to_owned())
    };
    let (_, response) = tokio::time::timeout(Duration::from_secs(5), async { tokio::join!(server, client) }).await.unwrap();
    response
}

#[tokio::test(flavor = "current_thread")]
async fn pid_endpoint_applies_reports_and_restores_gains() {
    let app = app();
    let (code, body) = request(app.clone(), "POST", "/api/pid", r#"{"kp":12.345,"ki":0.123,"kd":1.25}"#, "application/json").await;
    assert_eq!(code, 200, "{body}");
    let status: serde_json::Value = serde_json::from_str(&body).unwrap();
    assert_eq!((status["pid_kp"].as_f64(), status["pid_ki"].as_f64(), status["pid_kd"].as_f64()), (Some(12.345), Some(0.123), Some(1.25)));
    assert_eq!(status["settings_pending"], true);
    assert!(app.borrow().settings_dirty);
    let saved = app.borrow().settings();
    let record = crate::persist::encode(42, saved);
    let (_, decoded) = crate::persist::decode(&record).unwrap();
    assert_eq!(decoded, saved);
    let restored = AppState::new(Some(SafetyLimits { max_temperature_mc: 35_000, heater_watts: 100 }), decoded);
    assert_eq!(restored.control.pid_gains_milli(), (12_345, 123, 1_250));
    let (code, body) = request(app.clone(), "GET", "/metrics", "", "application/json").await;
    assert_eq!(code, 200);
    for line in ["birdburner_pid_kp 12.345\n", "birdburner_pid_ki 0.123\n", "birdburner_pid_kd 1.25\n"] {
        assert!(body.contains(line), "missing {line}");
    }
    app.borrow_mut().settings_dirty = false;
    let (code, _) = request(app.clone(), "POST", "/api/pid", r#"{"kp":12.345,"ki":0.123,"kd":1.25}"#, "application/json").await;
    assert_eq!(code, 200);
    assert!(!app.borrow().settings_dirty, "idempotent apply does not wear flash");
}

#[tokio::test(flavor = "current_thread")]
async fn pid_endpoint_rejects_bad_input_atomically() {
    for (body, content_type, expected) in [
        (r#"{"kp":100.001,"ki":0.1,"kd":0}"#.to_string(), "application/json", 422),
        (r#"{"kp":100.0001,"ki":0.1,"kd":0}"#.to_string(), "application/json", 422),
        (r#"{"kp":10,"ki":2.001,"kd":0}"#.to_string(), "application/json", 422),
        (r#"{"kp":10,"ki":0.1,"kd":200.001}"#.to_string(), "application/json", 422),
        (r#"{"kp":-1,"ki":0.1,"kd":0}"#.to_string(), "application/json", 422),
        (r#"{"kp":10,"ki":0.1}"#.to_string(), "application/json", 400),
        (r#"{"kp":10,"ki":0.1,"kd":0,"extra":1}"#.to_string(), "application/json", 400),
        (r#"{"kp":"10","ki":0.1,"kd":0}"#.to_string(), "application/json", 400),
        (r#"{"kp":NaN,"ki":0.1,"kd":0}"#.to_string(), "application/json", 400),
        (r#"{"kp":10,"ki":0.1,"kd":0}"#.to_string(), "text/plain", 415),
        (" ".repeat(129), "application/json", 413),
    ] {
        let app = app();
        let before = app.borrow().settings();
        let (code, result) = request(app.clone(), "POST", "/api/pid", &body, content_type).await;
        assert_eq!(code, expected, "{body} => {result}");
        assert_eq!(app.borrow().settings(), before);
        assert!(!app.borrow().settings_dirty);
    }
}

#[tokio::test(flavor = "current_thread")]
async fn browser_polling_through_reset_never_latches_error_and_real_failure_is_visible() {
    let app = app();
    let mut ms = 0;
    app.borrow_mut().control.restore_desired(true);
    for _ in 0..4 {
        app.borrow_mut().raw_reading(22_000, ms);
        ms += 5_000;
        app.borrow_mut().tick(ms);
    }
    assert_eq!(app.borrow().control.mode(), "pid");
    ms += 5_000;
    app.borrow_mut().tick(ms);
    app.borrow_mut().take_power_cycle();
    let (code, body) = request(app.clone(), "GET", "/api/status", "", "application/json").await;
    assert_eq!(code, 200);
    let status: serde_json::Value = serde_json::from_str(&body).unwrap();
    assert_eq!(status["sensor"], "resetting");
    assert_eq!(status["mode"], "pid");
    assert_eq!(status["fault"], serde_json::Value::Null);
    assert!(status["commanded_duty_pct"].as_f64().unwrap() > 0.0);
    // First valid raw read cancels the ladder, but the average is not ready yet. This used to
    // send sensor="error" even though the reset succeeded; explicitly test the transition.
    ms += 2_000;
    app.borrow_mut().raw_reading(22_100, ms);
    let (_, body) = request(app.clone(), "GET", "/api/status", "", "application/json").await;
    let status: serde_json::Value = serde_json::from_str(&body).unwrap();
    assert_eq!(status["sensor"], "recovering");
    assert_eq!(status["fault"], serde_json::Value::Null);
    for at in (ms..ms + 10_000).step_by(100) {
        if at % 700 == 0 { app.borrow_mut().raw_reading(22_100, at); }
        app.borrow_mut().tick(at);
    }
    let (_, body) = request(app.clone(), "GET", "/api/status", "", "application/json").await;
    let status: serde_json::Value = serde_json::from_str(&body).unwrap();
    assert_eq!(status["sensor"], "ok");
    assert_eq!(status["mode"], "pid");
    assert_eq!(status["fault"], serde_json::Value::Null);
    let until = ms + 80_000;
    for at in (ms + 10_000..until).step_by(100) {
        app.borrow_mut().tick(at);
        app.borrow_mut().take_power_cycle();
    }
    let (_, body) = request(app, "GET", "/api/status", "", "application/json").await;
    let status: serde_json::Value = serde_json::from_str(&body).unwrap();
    assert_eq!(status["fault"], "sensor_error");
    assert_eq!(status["mode"], "fault");
    assert_eq!(status["commanded_duty_pct"], 0.0);
}
