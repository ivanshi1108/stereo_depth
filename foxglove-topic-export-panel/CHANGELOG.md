# axera-roi-z-avg-export-panel version history

## 0.3.0

- Add Excel column `Raw Image` by loading matching `/camera/yuyv` frames.
- Add a default-enabled export toggle so `Raw Image` can be omitted from Excel when needed, and remember the last toggle state.
- Convert YUYV raw images to full-resolution PNG and embed them directly into exported Excel files.
- Show the matched raw-image count in the panel status summary.
- Derive the exported Excel filename from the opened MCAP when possible.

## 0.2.0

- Group ROI Excel exports under merged headers with `z_avg_mm` and percentage `confidence_avg` subcolumns.
- Rename the laser distance export field to `laser_distance_mm` to match the actual unit.
- Keep ROI headers stable even when `confidence_avg` is absent, leaving the related Excel cells blank.
- Tighten ROI export column widths for `z_avg_mm` and `confidence_avg`.

## 0.1.0

- Initial release with Axera ROI Z Avg Foxglove panel and Excel export.