# axera-roi-z-avg-export-panel

This Foxglove extension adds a custom panel dedicated to `/camera/roi_z_avg` for browsing all frames in the current MCAP file and exporting the ROI values as Excel.

## Features

- Load the full message range for `/camera/roi_z_avg` from the current MCAP
- Show one frame per row with `frame_timestamp_ns`, `laser_distance_mm`, and nine ROI groups in a fixed order
- Browse the loaded dataset page by page
- Inspect the raw JSON for the selected row
- Export Excel with grouped ROI headers: each ROI name spans `z_avg_mm` and percentage `confidence_avg` subcolumns
- Load matching `/camera/yuyv` frames from the same MCAP by `frame_timestamp_ns`
- Toggle whether matching `/camera/yuyv` images are exported to Excel; the option is enabled by default beside the export button and remembers the last selection
- Convert `/camera/yuyv` frames into full-resolution PNG images and embed them into Excel while displaying them scaled to fit the worksheet
- Load matching `/camera/yuyv` frames from the same MCAP by `frame_timestamp_ns`
- Toggle whether matching `/camera/yuyv` images are exported to Excel; the option is enabled by default beside the export button and remembers the last selection
- Convert `/camera/yuyv` frames into full-resolution PNG images and embed them into Excel while displaying them scaled to fit the worksheet
- Apply basic worksheet formatting such as frozen header, bold headers, borders, and numeric ROI cells

## Install Dependencies

```sh
cd msp/sample/stereo_depth/foxglove-topic-export-panel
npm install
```

## Build

```sh
npm run build
```

## Install Into Foxglove Desktop

```sh
npm run local-install
```

After installation or publication, add the `Axera ROI Z Avg Export` panel.

## Package

```sh
npm run package
```

This generates a `.foxe` package in the project directory.

## Notes

- Full-range loading depends on Foxglove `subscribeMessageRange()` support and is intended for offline file sources such as MCAP.
- Raw-image export expects the MCAP to contain `/camera/yuyv` messages whose timestamps match `/camera/roi_z_avg.frame_timestamp_ns`.
- The exported Excel filename tries to reuse the currently opened `.mcap` filename from the Foxglove window title, then falls back to a sanitized title or topic name.
- Embedded images keep the original sensor resolution in the PNG payload so they can be enlarged inside Excel for inspection.
- For very large topics, loading is still best-effort and may be limited by desktop/browser memory.