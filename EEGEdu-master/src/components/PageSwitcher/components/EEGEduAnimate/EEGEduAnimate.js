import React, { useState, useCallback } from "react";
import { catchError, multicast } from "rxjs/operators";

import { Card, Stack, TextContainer, RangeSlider, Select} from "@shopify/polaris";
import { Subject } from "rxjs";

import { zipSamples } from "muse-js";

import {
  bandpassFilter,
  epoch,
  fft,
  powerByBand
} from "@neurosity/pipes";

import { chartStyles } from "../chartOptions";

import { bandLabels } from "../../utils/chartUtils";

import sketchBands from './sketchBands'
import sketchTone from './sketchTone'
import sketchCube from './sketchCube'
import sketchFlock from './sketchFlock'
import sketchDraw from './sketchDraw'
import sketchFlock3D from './sketchFlock3D'

import P5Wrapper from 'react-p5-wrapper';

export function getSettings () {
  return {
    cutOffLow: 2,
    cutOffHigh: 20,
    interval: 16,
    bins: 256,
    duration: 128,
    srate: 256,
    name: 'Animate'
  }
};

window.eyeDirection = "closed";
window.awareness = 1;
window.attention = 1;

export function buildPipe(Settings) {
  if (window.subscriptionAnimate) window.subscriptionAnimate.unsubscribe();

  window.pipeAnimate$ = null;
  window.multicastAnimate$ = null;
  window.subscriptionAnimate = null;

  // Build Pipe
  window.pipeAnimate$ = zipSamples(window.source.eegReadings$).pipe(
    bandpassFilter({
      cutoffFrequencies: [Settings.cutOffLow, Settings.cutOffHigh],
      nbChannels: window.nchans }),
    epoch({
      duration: Settings.duration,
      interval: Settings.interval,
      samplingRate: Settings.srate
    }),
    fft({ bins: Settings.bins }),
    powerByBand(),
    catchError(err => {
      console.log(err);
    })
  );
  window.multicastAnimate$ = window.pipeAnimate$.pipe(
    multicast(() => new Subject())
  );
}

export function setup(setData, Settings) {
  console.log("Subscribing to " + Settings.name);

  if (window.multicastAnimate$) {
    window.subscriptionAnimate = window.multicastAnimate$.subscribe(data => {
      setData(animateData => {
        Object.values(animateData).forEach((channel, index) => {
            channel.datasets[0].data = [
              data.delta[index],
              data.theta[index],
              data.alpha[index],
              data.beta[index],
              data.gamma[index]
            ];
            channel.xLabels = bandLabels;
        });

        return {
          ch0: animateData.ch0,
          ch1: animateData.ch1,
          ch2: animateData.ch2,
          ch3: animateData.ch3,
          ch4: animateData.ch4
        };
      });
    });

    window.multicastAnimate$.connect();
    console.log("Subscribed to " + Settings.name);
  }
}

export function renderModule(channels) {
  function RenderCharts() {

    const bands = 'bands';
    const tone = 'tone';
    const cube = 'cube';
    const flock = 'flock';
    const draw = 'draw';
    const flock3d = 'flock3d';

    const chartTypes = [
      { label: bands, value: bands },
      { label: tone, value: tone },
      { label: cube, value: cube },
      { label: flock, value: flock },
      { label: draw, value: draw },
      { label: flock3d, value: flock3d }
    ];

    // for picking a new animation
    const [selectedAnimation, setSelectedAnimation] = useState(bands);
    const handleSelectChangeAnimation = useCallback(value => {
      setSelectedAnimation(value);
      console.log("Switching to: " + value);
    }, []);

    return Object.values(channels.data).map((channel, index) => {
      if (channel.datasets[0].data) {
        if (index === 1) {
          // console.log( channel.datasets[0].data[2])
          window.delta = channel.datasets[0].data[0];
          window.theta = channel.datasets[0].data[1];
          window.alpha = channel.datasets[0].data[2];
          window.beta  = channel.datasets[0].data[3];
          window.gamma = channel.datasets[0].data[4];
        }
      }

      let thisSketch = sketchTone;

      switch (selectedAnimation) {
        case bands:
          thisSketch = sketchBands;
          break
        case tone:
          thisSketch = sketchTone;
          break
        case cube:
          thisSketch = sketchCube;
          break
        case flock:
          thisSketch = sketchFlock;
          break
        case draw:
          thisSketch = sketchDraw;
          break
        case flock3d:
          thisSketch = sketchFlock3D;
          break
        default: console.log("Error on switch to " + selectedAnimation)
      }

      //only left frontal channel
      if (index === 1) {
        return (
          <React.Fragment key={'dum'}>
            <Card.Section>
              <P5Wrapper sketch={thisSketch}
                delta={window.delta}
                theta={window.theta}
                alpha={window.alpha}
                beta={window.beta}
                gamma={window.gamma}
              />
            </Card.Section>
          </React.Fragment>
        );
      } else {
        return null
      }
    });
  }

  function RenderEye(pos) {
    if (pos.x == "-1") {
      return (
        <svg width="100" height="100">
          <ellipse cx="50" cy="50" rx="45" ry="35" stroke="black" stroke-width="4" fill="#ffcc4d" />
          <line x1="5" y1="50" x2="95" y2="50" stroke="black" stroke-width="2"/>
        </svg>
      );
    }
    return (
      <svg width="100" height="100">
        <ellipse cx="50" cy="50" rx="45" ry="35" stroke="black" stroke-width="4" fill="white" />
        <circle cx={pos.x} cy={pos.y} r="20" fill="black" />
      </svg>
    );
  }

  function RenderEyes(state) {
    let pupilCenter = {
                        up: {
                          x: "50",
                          y: "35"
                        },
                        down: {
                          x: "50",
                          y: "65"
                        },
                        left: {
                          x: "30",
                          y: "50"
                        },
                        right: {
                          x: "70",
                          y: "50"
                        },
                        center: {
                          x: "50",
                          y: "50"
                        },
                        closed: {
                          x: "-1",
                          y: "-1",
                        }
                      };

    return (
      <React.Fragment>
        <Card.Section>
          <div style={{fontSize: '10rem', display: 'flex', alignItems: 'center', justifyContent: 'center'}}>
            {RenderEye(pupilCenter[state])}
            {RenderEye(pupilCenter[state])}
          </div>
        </Card.Section>
      </React.Fragment>
    );
  }

  function RenderAwareness(state) {
    let emoji = ['🙁', '😐', '🙂'];
    return (
      <React.Fragment>
        <Card.Section>
          <div style={{fontSize: '10rem', display: 'flex', alignItems: 'center', justifyContent: 'space-evenly', height: '100px'}}>
            {emoji[state]}
          </div>
        </Card.Section>
      </React.Fragment>
    );
  }

  function RenderAttention(state) {
    let emoji = ['❌', '✅'];
    return (
      <React.Fragment>
        <Card.Section>
          <div style={{fontSize: '10rem', display: 'flex', alignItems: 'center', justifyContent: 'space-evenly'}}>
            <i class="fas fa-road"></i>
            {emoji[state]}
          </div>
        </Card.Section>
      </React.Fragment>
    );
  }

  return (
    <React.Fragment>
    <Card title="Eyes">
      <Card.Section>
        <div>{RenderEyes(window.eyeDirection)}</div>
      </Card.Section>
    </Card>
    <Card title="Awareness">
      <Card.Section>
        <div>{RenderAwareness(window.awareness)}</div>
      </Card.Section>
    </Card>
    <Card title="Road Attention">
      <Card.Section>
        <div>{RenderAttention(window.attention)}</div>
      </Card.Section>
    </Card>
    </React.Fragment>
  );
}
