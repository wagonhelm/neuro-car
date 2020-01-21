import React from "react";
import { PageSwitcher } from "../PageSwitcher/PageSwitcher";
import { AppProvider, Page } from "@shopify/polaris";
import enTranslations from "@shopify/polaris/locales/en.json";
import * as translations from "./translations/en.json";

export function App() {
  return (
    <AppProvider i18n={enTranslations}>
      <Page title={translations.title}>
        <p>{translations.subtitle[0]}</p>
        <br/>
        <p>{translations.subtitle[1]}</p>
        <br/>
        <p>{translations.subtitle[2]}</p>
        <br/>
        <p>{translations.subtitle[3]}</p>
        <br/>
        <br/>
        <PageSwitcher />
      </Page>
    </AppProvider>
  );
}
