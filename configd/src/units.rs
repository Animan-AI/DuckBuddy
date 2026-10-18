//! Which of the robot's daemons are running, and which release each is running.
//!
//! Started as "is `padd` running?", reported alongside the pads because a connected pad and a dead
//! `padd` is the failure that looks like working hardware: the light on the controller is on, the
//! robot ignores it, and nothing in either place says why. That argument was never specific to
//! `padd` — a dead `btd` is a robot no phone can see, with the same silence — so it answers for
//! every unit a release manages.
//!
//! Asked of systemd rather than tracked: these are started, stopped and restarted by systemd, so
//! systemd is the only thing that knows. `configd` deliberately holds no opinion — it does not start
//! them, does not restart them, and reporting is the whole of its involvement.
//!
//! ## Which release is running
//!
//! Not asked of systemd and not inferred from `/proc`: **each daemon publishes its own identity at
//! startup**, to `/run/<service>/identity.json`, and this reads it. See
//! [`duck_ipc_proto::Identity`] for why that beats inspecting a process from outside — briefly, a
//! process knows its version, its git revision and its own exe, and needs no privilege to say so.
//!
//! What is left for systemd is the question only systemd can answer: whether the unit is running.
//! The two are read together because they mean different things apart. A published identity with a
//! stopped unit cannot happen — systemd deletes the runtime directory with the unit — but a *stopped
//! unit with no identity* and a *running daemon too old to publish one* both report nothing, and the
//! unit state is what distinguishes them.

use duck_ipc_proto as proto;

/// The unit that turns a pad into intents.
pub const PADD: &str = "padd.service";

/// Every unit a daemon release manages, in the order a reader wants them: the update engine, then
/// the robot, then the ones that depend on both.
///
/// Hardcoded rather than discovered, and that is a real limitation worth naming: a unit added to a
/// release and not to this list is invisible here. The alternative — asking systemd for everything
/// and filtering — reports units this project does not own, which is worse for a status line.
/// `scripts/install.sh` knows exactly these.
///
/// It has already cost once: `mediad` and `tofd` shipped units two releases before they were named
/// here, so the block a person reads after an update — the one that exists to say which daemon is
/// still on the old release — could not report either of them at all.
pub const MANAGED: [&str; 7] = [
    "updaterd.service",
    "robotd.service",
    "configd.service",
    "btd.service",
    "padd.service",
    "mediad.service",
    "tofd.service",
];

/// What systemd says about one unit. The narrow question, kept for `pad.status`.
pub async fn state(unit: &str) -> proto::UnitState {
    describe(unit).await.state
}

pub async fn all() -> Vec<proto::ServiceUnit> {
    let mut units = Vec::with_capacity(MANAGED.len());
    for unit in MANAGED {
        units.push(describe(unit).await);
    }
    units
}

#[cfg(target_os = "linux")]
pub async fn describe(unit: &str) -> proto::ServiceUnit {
    let state = match query(unit).await {
        Ok(state) => state,
        Err(e) => {
            // A warning, not an error: this is one line of a status report, and failing to read it
            // must not fail the report.
            tracing::warn!(error = %e, unit, "could not ask systemd about a unit");
            proto::UnitState::Unknown
        }
    };

    proto::ServiceUnit {
        identity: proto::read_identity(service_of(unit)),
        unit: unit.to_owned(),
        state,
    }
}

/// Off the board there is no systemd to ask, and inventing an answer would make a laptop look like a
/// robot with a broken daemon. The identity file is still read: it is an ordinary file, and a daemon
/// run by hand on a laptop publishes one.
#[cfg(not(target_os = "linux"))]
pub async fn describe(unit: &str) -> proto::ServiceUnit {
    proto::ServiceUnit {
        identity: proto::read_identity(service_of(unit)),
        unit: unit.to_owned(),
        state: proto::UnitState::Unknown,
    }
}

/// `btd.service` names the service `btd`, which is what it publishes under.
fn service_of(unit: &str) -> &str {
    unit.strip_suffix(".service").unwrap_or(unit)
}

#[cfg(target_os = "linux")]
async fn query(unit: &str) -> Result<proto::UnitState, String> {
    let bus = zbus::Connection::system()
        .await
        .map_err(|e| e.to_string())?;

    // `LoadUnit` rather than `GetUnit`: `GetUnit` fails for a unit systemd has not loaded, which is
    // indistinguishable from a unit that does not exist — and those are different answers here.
    // `LoadUnit` loads it if the file is there and fails only when it genuinely is not.
    let path: zbus::zvariant::OwnedObjectPath = match bus
        .call_method(
            Some("org.freedesktop.systemd1"),
            "/org/freedesktop/systemd1",
            Some("org.freedesktop.systemd1.Manager"),
            "LoadUnit",
            &(unit),
        )
        .await
    {
        Ok(reply) => reply.body().deserialize().map_err(|e| e.to_string())?,
        Err(e) => {
            // No such unit: a board on a release older than the one that added it. That is a fact
            // about the install, not a failure to report as one.
            tracing::debug!(error = %e, unit, "no such unit");
            return Ok(proto::UnitState::Absent);
        }
    };

    let active: String = property(&bus, &path, "org.freedesktop.systemd1.Unit", "ActiveState")
        .await?
        .try_into()
        .map_err(|e: zbus::zvariant::Error| e.to_string())?;

    let state = match active.as_str() {
        // `activating` counts as active: `padd` spends its first moments connecting to `robotd`, and
        // reporting that as "not running" would make a robot mid-boot look broken.
        "active" | "activating" | "reloading" => proto::UnitState::Active,
        // `failed` is inactive with a reason, and the reason is in the journal rather than here.
        // Collapsing them keeps this a status line rather than a diagnosis.
        "inactive" | "deactivating" | "failed" => proto::UnitState::Inactive,
        other => {
            tracing::warn!(state = other, unit, "unfamiliar unit state");
            proto::UnitState::Unknown
        }
    };

    Ok(state)
}

#[cfg(target_os = "linux")]
async fn property(
    bus: &zbus::Connection,
    path: &zbus::zvariant::OwnedObjectPath,
    interface: &str,
    name: &str,
) -> Result<zbus::zvariant::Value<'static>, String> {
    bus.call_method(
        Some("org.freedesktop.systemd1"),
        path,
        Some("org.freedesktop.DBus.Properties"),
        "Get",
        &(interface, name),
    )
    .await
    .map_err(|e| e.to_string())?
    .body()
    .deserialize::<zbus::zvariant::Value>()
    .map(|value| value.try_to_owned().map(Into::into))
    .map_err(|e| e.to_string())?
    .map_err(|e| e.to_string())
}
