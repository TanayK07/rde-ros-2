const assert = require("node:assert/strict");
const { EventEmitter } = require("node:events");
const { readFileSync } = require("node:fs");
const Module = require("node:module");
const path = require("node:path");
const { test } = require("node:test");

// Private compiled modules with fake timers and a fake ros2 process: no ROS,
// VS Code host, real timers, or subprocesses. Run after npm run test-compile.
function loadWithMocks(filename, mocks, globals) {
  const resolved = require.resolve(filename);
  const loaded = new Module(resolved, module);
  loaded.filename = resolved;
  loaded.paths = Module._nodeModulePaths(path.dirname(resolved));
  loaded.globals = globals;
  loaded.require = request => Object.hasOwn(mocks, request)
    ? mocks[request] : Module.prototype.require.call(loaded, request);
  loaded._compile("const { setTimeout, clearTimeout } = module.globals;\n" +
    readFileSync(resolved, "utf8"), resolved);
  return loaded.exports;
}

function deferred() {
  let resolve;
  let reject;
  const promise = new Promise((yes, no) => { resolve = yes; reject = no; });
  return { promise, resolve, reject };
}

const flush = () => new Promise(resolve => setImmediate(resolve));
const report = "average rate: 29.970\n\tmin: 0.032s max: 0.035s std dev: 0.00100s window: 31\n";
const rosEnv = { ROS_DISTRO: "fixture", PATH: "fixture ROS" };

class FakeProcess extends EventEmitter {
  stdout = new EventEmitter();
  stderr = new EventEmitter();
  exitCode = null;
  signalCode = null;
  kills = 0;
  kill() {
    this.kills++;
    this.signalCode = "SIGTERM";
    this.emit("exit", null, "SIGTERM");
    return true;
  }
  exit(code) {
    this.exitCode = code;
    this.emit("exit", code, null);
  }
}

function loadMonitor() {
  const h = { spawns: [], timers: new Set() };
  const globals = {
    setTimeout: (callback, ms) => { const timer = { callback, ms }; h.timers.add(timer); return timer; },
    clearTimeout: timer => h.timers.delete(timer),
  };
  h.expire = () => { for (const timer of [...h.timers]) { h.timers.delete(timer); timer.callback(); } };
  h.monitor = loadWithMocks("../out/src/ros/ros2/topic-monitor", {
    child_process: {
      spawn: (command, args, options) => {
        const process = new FakeProcess();
        h.spawns.push({ command, args, options, process });
        return process;
      },
      execFile: () => {},
    },
    "../../extension": { env: rosEnv, outputChannel: { appendLine() {} } },
    "./image-subscription": { ImageSubscriptionManager: class {}, PointCloudSubscriptionManager: class {} },
  }, globals);
  return h;
}

function loadSampler() {
  return loadWithMocks("../out/src/ros/topic-tree/topic-frequency-sampler", {}, { setTimeout, clearTimeout });
}

class FakeToken {
  isCancellationRequested = false;
  listeners = new Set();
  onCancellationRequested = listener => {
    this.listeners.add(listener);
    return { dispose: () => this.listeners.delete(listener) };
  };
  cancel() {
    this.isCancellationRequested = true;
    for (const listener of [...this.listeners]) { listener(); }
  }
}

test("ros2 topic hz stops at the first complete report, in the extension's environment", async () => {
  const h = loadMonitor();
  const result = h.monitor.getTopicFrequency("/camera");
  const [run] = h.spawns;
  assert.equal(run.command, "ros2");
  assert.deepEqual(run.args, ["topic", "hz", "/camera"]);
  assert.equal(run.options.env.PATH, rosEnv.PATH);
  assert.equal(run.options.env.PYTHONUNBUFFERED, "1", "Buffered Python output would be lost on kill");
  assert.equal([...h.timers][0].ms, 3000);

  run.process.stdout.emit("data", Buffer.from("average rate: 29.9"));
  await flush();
  assert.equal(run.process.kills, 0, "A partial line must not be parsed");
  run.process.stdout.emit("data", Buffer.from(report.slice("average rate: 29.9".length)));
  assert.equal((await result).averageRate, 29.97);
  assert.equal(run.process.kills, 1);
  assert.equal(h.timers.size, 0);
});

test("ros2 topic hz is killed when the window closes without messages", async () => {
  const h = loadMonitor();
  const result = h.monitor.getTopicFrequency("/quiet", 2);
  assert.equal([...h.timers][0].ms, 2000);
  h.expire();
  assert.equal(await result, null);
  assert.equal(h.spawns[0].process.kills, 1);
});

test("aborting stops ros2 topic hz, and an aborted signal never spawns it", async () => {
  const h = loadMonitor();
  const controller = new AbortController();
  const result = h.monitor.getTopicFrequency("/camera", 3, controller.signal);
  controller.abort();
  assert.equal(await result, null);
  assert.equal(h.spawns[0].process.kills, 1);
  assert.equal(h.timers.size, 0);

  assert.equal(await h.monitor.getTopicFrequency("/camera", 3, controller.signal), null);
  assert.equal(h.spawns.length, 1);
});

test("ros2 topic hz failures reject with its error output", async () => {
  const h = loadMonitor();
  const failed = h.monitor.getTopicFrequency("/custom");
  h.spawns[0].process.stderr.emit("data", Buffer.from("The message type 'custom/msg/X' is invalid\n"));
  h.spawns[0].process.exit(1);
  await assert.rejects(failed, /custom\/msg\/X/);
  assert.equal(h.spawns[0].process.kills, 0, "An exited process is not killed");

  const missing = h.monitor.getTopicFrequency("/camera");
  h.spawns[1].process.emit("error", new Error("spawn ros2 ENOENT"));
  await assert.rejects(missing, /ENOENT/);
});

test("the sampler caches a rate briefly and shares one run per topic", async () => {
  const { TopicFrequencySampler } = loadSampler();
  const runs = [];
  let now = 0;
  const sampler = new TopicFrequencySampler((name, signal) => {
    const run = deferred();
    runs.push({ name, signal, ...run });
    return run.promise;
  }, 10000, 2, () => now);

  const first = sampler.getFrequency("/camera", new FakeToken());
  const second = sampler.getFrequency("/camera", new FakeToken());
  assert.equal(runs.length, 1);
  runs[0].resolve({ averageRate: 30 });
  assert.deepEqual(await Promise.all([first, second]), [{ averageRate: 30 }, { averageRate: 30 }]);

  now = 9999;
  assert.deepEqual(await sampler.getFrequency("/camera"), { averageRate: 30 });
  assert.equal(runs.length, 1, "Cached rate");
  now = 10000;
  const expired = sampler.getFrequency("/camera");
  assert.equal(runs.length, 2, "Expired rate is sampled again");
  runs[1].resolve(null);
  assert.equal(await expired, null);
  assert.equal(await sampler.getFrequency("/camera"), null);
  assert.equal(runs.length, 2, "No-message results are cached too");
});

test("cancelling the last request stops the run and is not cached", async () => {
  const { TopicFrequencySampler } = loadSampler();
  const runs = [];
  const sampler = new TopicFrequencySampler((name, signal) => {
    const run = deferred();
    runs.push({ name, signal, ...run });
    return run.promise;
  });

  const kept = new FakeToken();
  const left = new FakeToken();
  const keptResult = sampler.getFrequency("/camera", kept);
  const leftResult = sampler.getFrequency("/camera", left);
  left.cancel();
  assert.equal(await leftResult, undefined);
  assert.equal(runs[0].signal.aborted, false, "Another hover still waits for this run");
  kept.cancel();
  assert.equal(await keptResult, undefined);
  assert.equal(runs[0].signal.aborted, true);
  runs[0].resolve(null);
  await flush();

  void sampler.getFrequency("/camera");
  assert.equal(runs.length, 2, "A cancelled run leaves nothing in the cache");

  const cancelled = new FakeToken();
  cancelled.cancel();
  assert.equal(await sampler.getFrequency("/other", cancelled), undefined);
  assert.equal(runs.length, 2, "A request cancelled up front never starts a run");
});

test("the sampler caps concurrent runs by stopping the oldest", async () => {
  const { TopicFrequencySampler } = loadSampler();
  const runs = [];
  const sampler = new TopicFrequencySampler((name, signal) => {
    const run = deferred();
    runs.push({ name, signal, ...run });
    return run.promise;
  }, 10000, 2);

  const oldest = sampler.getFrequency("/a");
  void sampler.getFrequency("/b");
  void sampler.getFrequency("/c");
  assert.deepEqual(runs.map(run => run.signal.aborted), [true, false, false]);
  runs[0].resolve(null);
  assert.equal(await oldest, undefined, "A stopped run is not reported as silent");

  sampler.clear();
  assert.deepEqual(runs.map(run => run.signal.aborted), [true, true, true]);
});

test("sampling errors reach the caller and are not cached", async () => {
  const { TopicFrequencySampler } = loadSampler();
  let calls = 0;
  const sampler = new TopicFrequencySampler(async () => {
    calls++;
    throw new Error("ros2 topic hz exited with code 1");
  });
  await assert.rejects(sampler.getFrequency("/camera"), /code 1/);
  await assert.rejects(sampler.getFrequency("/camera"), /code 1/);
  assert.equal(calls, 2);
});

test("hovering a topic fills its tooltip, and leaving stops the sample", async t => {
  const runs = [];
  class MarkdownString {
    value = "";
    appendMarkdown(text) { this.value += text; return this; }
  }
  const vscode = {
    EventEmitter: class { event = () => ({ dispose() {} }); fire() {} dispose() {} },
    TreeItem: class { constructor(label) { this.label = label; } },
    ThemeIcon: class {},
    MarkdownString,
    TreeItemCollapsibleState: { None: 0 },
    TreeItemCheckboxState: { Unchecked: 0, Checked: 1 },
    window: { onDidChangeWindowState: () => ({ dispose() {} }) },
  };
  const globals = { setTimeout, clearTimeout };
  const items = loadWithMocks("../out/src/ros/topic-tree/topic-tree-item", { vscode }, globals);
  const { TopicTreeDataProvider } = loadWithMocks("../out/src/ros/topic-tree/topic-tree-provider", {
    vscode,
    "./topic-tree-item": items,
    "../ros2/topic-monitor": {
      getTopicInfo: async name => ({ name, type: "", publisherCount: 1, subscriberCount: 2 }),
      getTopicFrequency: (name, durationSec, signal) => {
        const run = deferred();
        runs.push({ name, durationSec, signal, ...run });
        return run.promise;
      },
    },
  }, globals);
  const logs = [];
  const provider = new TopicTreeDataProvider({ subscriptions: [] }, { appendLine: text => logs.push(text) }, () => {});
  t.after(() => provider.dispose());
  const topicItem = () => items.TopicTreeItem.createTopicItem({ name: "/camera", type: "sensor_msgs/msg/Image" });

  const item = topicItem();
  assert.equal(provider.getTreeItem(item).tooltip, undefined, "Left unset so VS Code resolves it on hover");
  const hovered = provider.resolveTreeItem(item, item, new FakeToken());
  assert.equal(runs[0].durationSec, 3);
  runs[0].resolve({ averageRate: 29.97 });
  const tooltip = (await hovered).tooltip.value;
  assert.match(tooltip, /Frequency: 29\.97 Hz/);
  assert.match(tooltip, /Publishers: 1/);

  provider.invalidateEnvironment();
  const leaving = new FakeToken();
  const left = topicItem();
  const leftResult = provider.resolveTreeItem(left, left, leaving);
  leaving.cancel();
  await leftResult;
  assert.equal(runs[1].signal.aborted, true);
  assert.equal(left.tooltip, undefined);

  const quiet = topicItem();
  const quietResult = provider.resolveTreeItem(quiet, quiet, new FakeToken());
  runs[2].resolve(null);
  assert.match((await quietResult).tooltip.value, /Frequency: no recent messages/);

  provider.invalidateEnvironment();
  const failing = topicItem();
  const failingResult = provider.resolveTreeItem(failing, failing, new FakeToken());
  runs[3].reject(new Error("spawn ros2 ENOENT"));
  const failedTooltip = (await failingResult).tooltip.value;
  assert.doesNotMatch(failedTooltip, /Frequency/);
  assert.match(failedTooltip, /Publishers: 1/, "The rest of the tooltip still renders");
  assert.match(logs.join("\n"), /Error measuring frequency for \/camera: spawn ros2 ENOENT/);
});
