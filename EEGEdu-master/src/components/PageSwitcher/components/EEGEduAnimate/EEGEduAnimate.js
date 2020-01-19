import React from "react";
import { catchError, multicast } from "rxjs/operators";

import { Card } from "@shopify/polaris";
import { Subject } from "rxjs";

import { zipSamples } from "muse-js";

import {
  bandpassFilter,
  epoch,
  fft,
  powerByBand
} from "@neurosity/pipes";

import { bandLabels } from "../../utils/chartUtils";

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
function loadVariables() {
    var xmlhttp = new XMLHttpRequest();
    xmlhttp.onreadystatechange = () => {
        if (this.readyState == 4 && this.status == 200) {
            var vars = JSON.parse(this.responseText);
            console.table(vars);
        }
    };
    xmlhttp.open("GET", "/home/giancarlo/vars", true);
    xmlhttp.send();
}

setInterval(loadVariables, 3000);

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
  function RenderEye(pos) {
    if (pos.x === "-1") {
      return (
        <svg width="100" height="100">
          <ellipse cx="50" cy="50" rx="45" ry="35" stroke="black" strokeWidth="4" fill="#ffcc4d" />
          <line x1="5" y1="50" x2="95" y2="50" stroke="black" strokeWidth="2"/>
        </svg>
      );
    }
    return (
      <svg width="100" height="100">
        <ellipse cx="50" cy="50" rx="45" ry="35" stroke="black" strokeWidth="4" fill="white" />
        <circle cx={pos.x} cy={pos.y} r="20" fill="black" />
      </svg>
    );
  }

  function RenderEyes() {
    let state = window.eyeDirection;
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

  function RenderAwareness() {
    let state = window.awareness;
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

  function RenderAttention() {
    let state = window.attention;
    let emoji = ['❌', '✅'];
    return (
      <React.Fragment>
        <Card.Section>
          <div style={{fontSize: '10rem', display: 'flex', alignItems: 'center', justifyContent: 'space-evenly'}}>
            <i className="fas fa-road"></i>
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
        <div>{RenderEyes()}</div>
      </Card.Section>
    </Card>
    <Card title="Awareness">
      <Card.Section>
        <div>{RenderAwareness()}</div>
      </Card.Section>
    </Card>
    <Card title="Road Attention">
      <Card.Section>
        <div>{RenderAttention()}</div>
      </Card.Section>
    </Card>
    </React.Fragment>
  );
}
