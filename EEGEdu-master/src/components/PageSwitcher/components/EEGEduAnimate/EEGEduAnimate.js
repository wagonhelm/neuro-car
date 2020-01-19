import React from "react";

import { Card } from "@shopify/polaris";

import { chartStyles, generalOptions } from "../chartOptions";

import { Line } from "react-chartjs-2";

const io = require('socket.io-client');
const mister_yeet_sock = io("http://34.94.143.73:9001");
console.log("Got a yeety socket over here coming in hawt!", mister_yeet_sock);

mister_yeet_sock.emit('gimme_sashimi')
console.log("Entered the sashimi restaurant!")

mister_yeet_sock.on('eyes_yeet', (fresh_yeet) => {
    console.log("Got some yeet's lookin' over here: ", fresh_yeet);

    // fresh_yeet is 1 for open and aware, 2 for closed and not aware

    if(fresh_yeet === 1){
        window.eyeDirection = "center";
    }else if(fresh_yeet === 0){
        window.eyeDirection = "closed";
    }else{
        alert("Got some bad, gross, green yeet! :'(");
        console.log("Ew, eyes yeet was: ", fresh_yeet)
    }
});

mister_yeet_sock.on('fatigue_yeet', (fresh_yeet) => {
    console.log("Got some tired yeets over here: ", fresh_yeet);

    // fresh_yeet is 1 for open and aware, 2 for closed and not aware

    if(fresh_yeet === 2){
        window.awareness = 0
    }else if(fresh_yeet === 1) {
        window.awareness = 1;
    }else if(fresh_yeet === 0) {
        window.awareness = 2;
    }else{
        alert("Got some bad, gross, green yeet! :'(");
        console.log("Ew, fatigue yeet was: ", fresh_yeet)
    }
});

mister_yeet_sock.on('attention_yeet', (fresh_yeet) => {
    console.log("Got some attentive yeets over here: ", fresh_yeet);

    // fresh_yeet is 1 for open and aware, 2 for closed and not aware

    if(fresh_yeet === 1){
        window.attention = 1;
    }else if(fresh_yeet === 0) {
        window.attention = 0;
    }else{
        alert("Got some bad, gross, green yeet! :'(");
        console.log("Ew, attention yeet was: ", fresh_yeet)
    }
});


window.eyeDirection = "closed";
window.awareness = 1;
window.attention = 1;
window.latestSashimi = [[],[],[],[]]
window.sashimiLabels = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30];

mister_yeet_sock.on('cooked_sashimi', (sashimi) => {
    window.latestSashimi[0] = sashimi.psd[0]
    window.latestSashimi[1] = sashimi.psd[1]
    window.latestSashimi[2] = sashimi.psd[2]
    window.latestSashimi[3] = sashimi.psd[3]

})

function loadVariables() {
    var xmlhttp = new XMLHttpRequest();
    xmlhttp.onreadystatechange = () => {
        if (this.readyState === 4 && this.status === 200) {
            var vars = JSON.parse(this.responseText);
            console.table(vars);
        }
    };
    xmlhttp.open("GET", "/home/giancarlo/vars", true);
    xmlhttp.send();
}

setInterval(loadVariables, 3000);

export function renderModule() {
    function renderCharts() {
        let vertLim = Math.floor(Math.max(...[].concat.apply([], [window.latestSashimi[0],
            window.latestSashimi[1],
            window.latestSashimi[2],
            window.latestSashimi[3]])
        ));
        const options = {
            ...generalOptions,
            scales: {
                xAxes: [
                    {
                        scaleLabel: {
                            ...generalOptions.scales.xAxes[0].scaleLabel,
                            labelString: "Frequency (Hz)"
                        }
                    }
                ],
                yAxes: [
                    {
                        scaleLabel: {
                            ...generalOptions.scales.yAxes[0].scaleLabel,
                            labelString: "Power (μV²)"
                        },
                        ticks: {
                            max: vertLim,
                            min: vertLim * -1
                        }
                    }
                ]
            },
            elements: {
                point: {
                    radius: 3
                }
            },
            title: {
                ...generalOptions.title,
                text: 'Spectra data from each electrode'
            },
            legend: {
                display: true
            }
        };


        if (window.latestSashimi[0].length > 0) {
            const newData = {
                datasets: [{
                    label: "TP9",
                    borderColor: 'rgba(217,95,2)',
                    data: window.latestSashimi[0].map(function (x) {
                        return x * -1
                    }),
                    fill: false
                }, {
                    label: "AF7",
                    borderColor: 'rgba(27,158,119)',
                    data: window.latestSashimi[1].map(function (x) {
                        return x * -1
                    }),
                    fill: false
                }, {
                    label: "AF8",
                    borderColor: 'rgba(117,112,179)',
                    data: window.latestSashimi[2].map(function (x) {
                        return x + 0
                    }),
                    fill: false
                }, {
                    label: "TP10",
                    borderColor: 'rgba(231,41,138)',
                    data: window.latestSashimi[3].map(function (x) {
                        return x + 0
                    }),
                    fill: false
                }],
                xLabels: window.sashimiLabels
            }

            return (
                <Card.Section key={"Card_" + 1}>
                    <Line key={"Line_" + 1} data={newData} options={options}/>
                </Card.Section>
            );
        }
    }
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
    <Card title="Fatigue">
      <Card.Section>
        <div>{RenderAwareness()}</div>
      </Card.Section>
    </Card>
    <Card title="Attention">
      <Card.Section>
        <div>{RenderAttention()}</div>
      </Card.Section>
    </Card>
    <Card title="EEG Data">
        <Card.Section>
            <div style={chartStyles.wrapperStyle.style}>{renderCharts()}</div>
        </Card.Section>
    </Card>
    </React.Fragment>
  );
}
