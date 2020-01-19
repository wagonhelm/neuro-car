import React, { useState, useCallback } from "react";
import { MuseClient } from "muse-js";
import { Select, Card, Stack, Button, ButtonGroup, Checkbox } from "@shopify/polaris";

import { mockMuseEEG } from "./utils/mockMuseEEG";
import * as translations from "./translations/en.json";
import * as generalTranslations from "./components/translations/en";
import { emptyAuxChannelData } from "./components/chartOptions";

import * as funAnimate from "./components/EEGEduAnimate/EEGEduAnimate";

const animate = translations.types.animate;

export function PageSwitcher() {

  // For auxEnable settings
  const [checked, setChecked] = useState(false);
  const handleChange = useCallback((newChecked) => setChecked(newChecked), []);
  window.enableAux = checked;
  if (window.enableAux) {
    window.nchans = 5;
  } else {
    window.nchans = 4;
  }
  let showAux = true; // if it is even available to press (to prevent in some modules)

  // data pulled out of multicast$
  const [animateData, setAnimateData] = useState(emptyAuxChannelData);

  // pipe settings
  const [animateSettings, setAnimateSettings] = useState(funAnimate.getSettings);

  // connection status
  const [status, setStatus] = useState(generalTranslations.connect);

  // for picking a new module
  const [selected, setSelected] = useState(animate);
  const handleSelectChange = useCallback(value => {
    value = "7. Brain Controlled Animation";
    setSelected(value);

    console.log("Switching to: " + value);

    if (window.subscriptionAnimate) window.subscriptionAnimate.unsubscribe();

    subscriptionSetup(value);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  // for popup flag when recording
  const [recordPop, setRecordPop] = useState(false);
  const recordPopChange = useCallback(() => setRecordPop(!recordPop), [recordPop]);

  // for popup flag when recording 2nd condition
  const [recordTwoPop, setRecordTwoPop] = useState(false);
  const recordTwoPopChange = useCallback(() => setRecordTwoPop(!recordTwoPop), [recordTwoPop]);

  switch (selected) {
    case animate:
      showAux = false;
      break
    default:
      console.log("Error on showAux");
  }


  const chartTypes = [
    { label: animate, value: animate },
  ];

  function buildPipes(value) {
    funAnimate.buildPipe(animateSettings);
  }

  function subscriptionSetup(value) {
    switch (value) {
      case animate:
        funAnimate.setup(setAnimateData, animateSettings);
        break;
      default:
        console.log(
          "Error on handle Subscriptions. Couldn't switch to: " + value
        );
    }
  }

  async function connect() {
    try {
      if (window.debugWithMock) {
        // Debug with Mock EEG Data
        setStatus(generalTranslations.connectingMock);
        window.source = {};
        window.source.connectionStatus = {};
        window.source.connectionStatus.value = true;
        window.source.eegReadings$ = mockMuseEEG(256);
        setStatus(generalTranslations.connectedMock);
      } else {
        // Connect with the Muse EEG Client
        setStatus(generalTranslations.connecting);
        window.source = new MuseClient();
        window.source.enableAux = window.enableAux;
        await window.source.connect();
        await window.source.start();
        window.source.eegReadings$ = window.source.eegReadings;
        setStatus(generalTranslations.connected);
      }
      if (
        window.source.connectionStatus.value === true &&
        window.source.eegReadings$
      ) {
        buildPipes(selected);
        subscriptionSetup(selected);
      }
    } catch (err) {
      setStatus(generalTranslations.connect);
      console.log("Connection error: " + err);
    }
  }

  function refreshPage(){
    window.location.reload();
  }

  function pipeSettingsDisplay() {
    switch(selected) {
      case animate:
        return (
          funAnimate.renderSliders(setAnimateData, setAnimateSettings, status, animateSettings)
        );
      default: console.log('Error rendering settings display');
    }
  }

  function renderModules() {
    switch (selected) {
      case animate:
        return <funAnimate.renderModule data={animateData} />;
      default:
        console.log("Error on renderCharts switch.");
    }
  }

  function renderRecord() {
    switch (selected) {
      case animate:
        return null
      default:
        console.log("Error on renderRecord.");
    }
  }

  // Render the entire page using above functions
  return (
    <React.Fragment>
      <Card sectioned>
        <Stack>
          <ButtonGroup>
            <Button
              primary={status === generalTranslations.connect}
              disabled={status !== generalTranslations.connect}
              onClick={() => {
                window.debugWithMock = false;
                connect();
              }}
            >
              {status}
            </Button>
            <Button
              disabled={status !== generalTranslations.connect}
              onClick={() => {
                window.debugWithMock = true;
                connect();
              }}
            >
              {status === generalTranslations.connect ? generalTranslations.connectMock : status}
            </Button>
            <Button
              destructive
              onClick={refreshPage}
              primary={status !== generalTranslations.connect}
              disabled={status === generalTranslations.connect}
            >
              {generalTranslations.disconnect}
            </Button>
          </ButtonGroup>
          <Checkbox
            label="Enable Muse Auxillary Channel"
            checked={checked}
            onChange={handleChange}
            disabled={!showAux || status !== generalTranslations.connect}
          />
        </Stack>
      </Card>
      {pipeSettingsDisplay()}
      {renderModules()}
      {renderRecord()}
    </React.Fragment>
  );
}
