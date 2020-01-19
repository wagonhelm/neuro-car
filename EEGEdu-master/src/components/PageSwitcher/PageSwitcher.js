import React, { useState } from "react";

import * as translations from "./translations/en.json";
import { emptyAuxChannelData } from "./components/chartOptions";

import * as funAnimate from "./components/EEGEduAnimate/EEGEduAnimate";

const animate = translations.types.animate;

export function PageSwitcher() {

  // For auxEnable settings
  const checked = useState(false)[0];
  window.enableAux = checked;
  if (window.enableAux) {
    window.nchans = 5;
  } else {
    window.nchans = 4;
  }
  let showAux = true; // if it is even available to press (to prevent in some modules)

  // data pulled out of multicast$
  const animateData = useState(emptyAuxChannelData)[0];

  // for picking a new module
  const selected= useState(animate)[0];

  switch (selected) {
    case animate:
      showAux = false;
      break
    default:
      console.log("Error on showAux");
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
      {renderModules()}
      {renderRecord()}
    </React.Fragment>
  );
}
