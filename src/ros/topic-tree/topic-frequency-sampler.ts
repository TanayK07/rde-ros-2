// Copyright (c) Ranch Hand Robotics. All rights reserved.
// Licensed under the MIT License.

import * as vscode from 'vscode';
import { TopicMetrics } from '../ros2/topic-types';

/**
 * Measures one topic's rate. Resolves null when no messages arrived and must stop when signal aborts.
 */
export type TopicFrequencySource = (topicName: string, signal: AbortSignal) => Promise<TopicMetrics | null>;

interface FrequencySample {
  controller: AbortController;
  promise: Promise<TopicMetrics | null | undefined>;
  waiters: number;
}

/**
 * Samples topic rates only while someone is looking at a topic.
 * Results are cached briefly, requests for the same topic share one run,
 * and starting a run beyond the limit stops the oldest one.
 */
export class TopicFrequencySampler {
  private readonly cache = new Map<string, { metrics: TopicMetrics | null, updatedAt: number }>();
  private readonly samples = new Map<string, FrequencySample>();

  constructor(
    private readonly source: TopicFrequencySource,
    private readonly cacheDurationMs = 10000,
    private readonly maxConcurrentSamples = 2,
    private readonly now: () => number = Date.now
  ) {}

  /**
   * Resolves the topic's rate, null when no messages arrived, or undefined when the request was cancelled.
   */
  public async getFrequency(topicName: string, token?: vscode.CancellationToken): Promise<TopicMetrics | null | undefined> {
    const cached = this.cache.get(topicName);
    if (cached && this.now() - cached.updatedAt < this.cacheDurationMs) {
      return cached.metrics;
    }

    if (token?.isCancellationRequested) {
      return undefined;
    }

    const sample = this.samples.get(topicName) ?? this.start(topicName);
    sample.waiters++;

    return new Promise<TopicMetrics | null | undefined>((resolve, reject) => {
      const cancellation = token?.onCancellationRequested(() => {
        cancellation?.dispose();
        this.release(topicName, sample);
        resolve(undefined);
      });

      sample.promise.then(metrics => {
        cancellation?.dispose();
        resolve(metrics);
      }, error => {
        cancellation?.dispose();
        reject(error);
      });
    });
  }

  /**
   * Stop every run and forget cached rates, e.g. after the ROS environment changes.
   */
  public clear(): void {
    for (const [topicName, sample] of Array.from(this.samples)) {
      this.stop(topicName, sample);
    }
    this.cache.clear();
  }

  private start(topicName: string): FrequencySample {
    // Hover moves on quickly, so the oldest run is the one nobody is looking at any more.
    while (this.samples.size >= this.maxConcurrentSamples) {
      const [oldestName, oldest] = this.samples.entries().next().value as [string, FrequencySample];
      this.stop(oldestName, oldest);
    }

    const controller = new AbortController();
    const sample: FrequencySample = { controller, promise: Promise.resolve(undefined), waiters: 0 };
    const done = (): void => {
      if (this.samples.get(topicName) === sample) {
        this.samples.delete(topicName);
      }
    };

    sample.promise = this.source(topicName, controller.signal).then(metrics => {
      done();
      if (controller.signal.aborted) {
        return undefined;
      }
      this.cache.set(topicName, { metrics, updatedAt: this.now() });
      return metrics;
    }, error => {
      done();
      if (controller.signal.aborted) {
        return undefined;
      }
      throw error;
    });

    this.samples.set(topicName, sample);
    return sample;
  }

  private release(topicName: string, sample: FrequencySample): void {
    sample.waiters--;
    if (sample.waiters <= 0) {
      this.stop(topicName, sample);
    }
  }

  private stop(topicName: string, sample: FrequencySample): void {
    if (this.samples.get(topicName) === sample) {
      this.samples.delete(topicName);
    }
    sample.controller.abort();
  }
}
