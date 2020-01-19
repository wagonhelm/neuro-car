import React from "react";
import ReactDOM from "react-dom";
import { App } from "./components/App/App";

function render() {
    ReactDOM.render(<App />, document.getElementById("root"));
}

setInterval(render, 500);
