import { ExtensionContext } from "@foxglove/extension";

import { initTopicRangeExportPanel } from "./TopicRangeExportPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({
    name: "axera-roi-z-avg-export-panel",
    initPanel: initTopicRangeExportPanel,
  });
}