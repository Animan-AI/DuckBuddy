"use strict";

const byId = id => document.getElementById(id);
const actions = [...document.querySelectorAll("[data-action]")];
const buttons = [...document.querySelectorAll("[data-direction]")];
const client = Array.from(crypto.getRandomValues(new Uint8Array(16)), byte => byte.toString(16).padStart(2, "0")).join("");
const keys = {w:"forward", s:"back", a:"left", d:"right", q:"turn-left", e:"turn-right",
              ArrowUp:"forward", ArrowDown:"back", ArrowLeft:"turn-left", ArrowRight:"turn-right"};
let sequence = 0, armed = false, direction = null, driverInput = null, pulse = null, status = {};
let watching = false, videoFrame = null, photoFrame = null, cameraVersion = 0;

function message(text, error = false) {
  const el = byId("command-result");
  el.textContent = text;
  el.classList.toggle("error", error);
}

function post(path, data, keepalive = false) {
  return fetch("/api/console/" + path, {
    method: "POST", keepalive, headers: {"Content-Type": "application/json"}, body: JSON.stringify(data)
  });
}

async function command(action, extra = {}, keepalive = false) {
  const response = await post("action", {action, client, ...extra}, keepalive);
  const result = await response.json();
  if (!result.ok) throw new Error(result.reason || "操作失败");
}

function refreshButtons() {
  for (const button of actions) {
    const name = button.dataset.action;
    const available = name === "relax" ? status.online : status.actions?.[name];
    button.disabled = !available;
    button.title = available ? "" : "控制服务或所需动作模型还未就绪";
  }
  for (const button of buttons) {
    button.disabled = !(armed && status.actions?.walk);
    button.classList.toggle("pressed", button.dataset.direction === direction);
  }
  byId("drive-status").textContent = armed ? "驾驶已启用 · 松开方向即停" : "启用行走后按住方向移动，松手停止";
}

function renderModels(models) {
  const tbody = byId("model-list");
  tbody.replaceChildren();
  if (!models.length) {
    const cell = tbody.insertRow().insertCell();
    cell.colSpan = 3;
    cell.textContent = "还没有配置动作模型";
    return;
  }
  const labels = {loaded:"已加载", configured:"已配置，等待加载", missing:"文件缺失"};
  for (const item of models) {
    const row = tbody.insertRow();
    row.insertCell().textContent = item.label;
    row.insertCell().textContent = item.file;
    const state = row.insertCell();
    state.textContent = labels[item.state] || "—";
    state.dataset.state = item.state;
  }
}

async function pollStatus() {
  try {
    status = await (await fetch("/api/console", {cache:"no-store"})).json();
    byId("robot-status").textContent = status.ready ? "已就绪" : status.online ? "未就绪" : "未运行";
    const battery = status.battery || {};
    byId("battery").textContent = Number.isFinite(battery.volts)
      ? [battery.volts.toFixed(2) + " V", Number.isFinite(battery.percent) ? Math.round(battery.percent) + "%" : ""].filter(Boolean).join(" · ") : "—";
    byId("temperature").textContent = Number.isFinite(status.temperature) ? Math.round(status.temperature) + " °C" : "—";
    byId("model-count").textContent = status.models.filter(x => x.state === "loaded").length;
    const note = byId("robot-note");
    note.textContent = status.reason || "";
    note.hidden = !status.reason;
    if (armed && (!status.controls || !status.actions?.walk)) {
      armed = false;
      stopDrive();
    }
    renderModels(status.models);
  } catch {
    status = {};
    armed = false;
    stopDrive();
    byId("robot-status").textContent = "连接中断";
  }
  refreshButtons();
  setTimeout(pollStatus, 1200);
}

function stopDrive(force = false) {
  const wasMoving = direction !== null;
  clearTimeout(pulse);
  direction = driverInput = null;
  refreshButtons();
  if (wasMoving || force) command("stop", {sequence: ++sequence}, true).catch(error => message(error.message, true));
}

function startDrive(next, input) {
  if (direction || !armed || !status.actions?.walk) return;
  direction = next;
  driverInput = input;
  refreshButtons();
  async function send() {
    if (direction !== next || driverInput !== input) return;
    try {
      await command("move", {direction:next, sequence:++sequence});
    } catch (error) {
      stopDrive();
      message(error.message, true);
      return;
    }
    if (direction === next && driverInput === input) pulse = setTimeout(send, 110);
  }
  send();
}

for (const button of buttons) {
  button.addEventListener("pointerdown", event => {
    if (event.button !== 0) return;
    event.preventDefault();
    button.setPointerCapture(event.pointerId);
    startDrive(button.dataset.direction, event.pointerId);
  });
  for (const event of ["pointerup", "pointercancel", "lostpointercapture"]) {
    button.addEventListener(event, e => { if (driverInput === e.pointerId) stopDrive(); });
  }
}

window.addEventListener("keydown", event => {
  if (!armed || event.ctrlKey || event.metaKey || event.altKey ||
      /^(INPUT|SELECT|TEXTAREA)$/.test(event.target.tagName) || event.target.isContentEditable) return;
  const key = event.key.length === 1 ? event.key.toLowerCase() : event.key;
  if (key === " ") {
    if (/^(BUTTON|A)$/.test(event.target.tagName)) return;
    event.preventDefault();
    stopDrive(true);
  } else if (keys[key]) {
    event.preventDefault();
    if (!event.repeat) startDrive(keys[key], key);
  }
});
window.addEventListener("keyup", event => {
  const key = event.key.length === 1 ? event.key.toLowerCase() : event.key;
  if (key === driverInput) stopDrive();
});
window.addEventListener("blur", () => stopDrive());
window.addEventListener("pagehide", () => stopDrive());
document.addEventListener("visibilitychange", () => { if (document.hidden) stopDrive(); });
byId("stop").onclick = () => stopDrive(true);

for (const button of actions) {
  button.onclick = async () => {
    if (!window.confirm(button.dataset.action === "relax" ? "请先托稳鸭子，确认后会卸力。" : "确认鸭子已托稳，或动作范围内有足够空间？")) return;
    stopDrive();
    button.disabled = true;
    try {
      await command(button.dataset.action);
      armed = button.dataset.action === "walk";
      if (armed) status.controls = true;
      message("已发送" + button.textContent + "指令");
    } catch (error) {
      message(error.message, true);
    } finally {
      refreshButtons();
    }
  };
}

function setImage(element, data, previous) {
  const next = URL.createObjectURL(data);
  element.src = next;
  if (previous) URL.revokeObjectURL(previous);
  return next;
}

async function cameraFeed(version) {
  if (!watching || version !== cameraVersion) return;
  if (!document.hidden) {
    try {
      const response = await fetch("/api/console/preview", {cache:"no-store"});
      if (!response.ok) throw new Error((await response.json()).reason);
      const data = await response.blob();
      if (watching && version === cameraVersion) {
        videoFrame = setImage(byId("preview"), data, videoFrame);
        byId("preview").hidden = false;
        byId("camera-placeholder").hidden = true;
        byId("camera-status").textContent = "预览中";
      }
    } catch (error) {
      if (watching && version === cameraVersion) {
        byId("preview").hidden = true;
        byId("camera-placeholder").hidden = false;
        byId("camera-placeholder").textContent = error.message || "等待摄像头";
        byId("camera-status").textContent = "等待画面";
      }
    }
  }
  if (watching && version === cameraVersion) setTimeout(() => cameraFeed(version), 600);
}

async function startCamera() {
  const response = await post("camera", {});
  const result = await response.json();
  if (!result.ok) throw new Error(result.reason);
}

byId("camera-switch").onclick = async () => {
  const button = byId("camera-switch");
  button.disabled = true;
  try {
    if (watching) {
      watching = false;
      ++cameraVersion;
      if (videoFrame) URL.revokeObjectURL(videoFrame);
      videoFrame = null;
      byId("preview").hidden = true;
      byId("camera-placeholder").hidden = false;
      byId("camera-placeholder").textContent = "开启预览后显示机头画面";
      byId("camera-status").textContent = "预览已关闭";
      button.textContent = "开启预览";
    } else {
      await startCamera();
      watching = true;
      byId("camera-status").textContent = "等待画面";
      button.textContent = "关闭预览";
      cameraFeed(++cameraVersion);
    }
  } catch (error) {
    byId("camera-status").textContent = error.message;
  } finally {
    button.disabled = false;
  }
};

byId("photo").onclick = async () => {
  const button = byId("photo");
  button.disabled = true;
  const version = ++cameraVersion;
  byId("camera-status").textContent = "拍照中";
  try {
    await startCamera();
    const response = await post("photo", {});
    if (!response.ok) throw new Error((await response.json()).reason);
    photoFrame = setImage(byId("photo-image"), await response.blob(), photoFrame);
    const download = byId("save-photo");
    download.href = photoFrame;
    download.download = "DuckBuddy-" + new Date().toISOString().replaceAll(":", "-") + ".jpg";
    download.hidden = byId("photo-frame").hidden = false;
    byId("camera-status").textContent = "已拍照";
  } catch (error) {
    byId("camera-status").textContent = error.message;
  } finally {
    button.disabled = false;
    if (watching && version === cameraVersion) cameraFeed(version);
  }
};

pollStatus();
