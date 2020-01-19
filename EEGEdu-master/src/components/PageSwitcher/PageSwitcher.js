import React, { useState } from "react";

import { emptyAuxChannelData } from "./components/chartOptions";

import * as funAnimate from "./components/EEGEduAnimate/EEGEduAnimate";

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
  showAux = false;

  function renderModules() {
    return <funAnimate.renderModule data={animateData} />;
  }

  function renderRecord() {
    return null
  }

  // Render the entire page using above functions
  return (
    <React.Fragment>
      {renderModules()}
      {renderRecord()}
    </React.Fragment>
  );
}
