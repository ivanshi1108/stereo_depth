import {
  Immutable,
  MessageEvent,
  PanelExtensionContext,
  Time,
} from "@foxglove/extension";
import ExcelJS from "exceljs";
import { ReactElement, useEffect, useLayoutEffect, useRef, useState } from "react";
import { createRoot } from "react-dom/client";

type PanelState = {
  pageSize?: number;
  includeRawImagesInExport?: boolean;
};

type RoiItem = {
  name?: string;
  z_avg_mm?: number | null;
  confidence_avg?: number | null;
};

type RoiZAvgMessage = {
  frame_timestamp_ns?: number | string;
  items?: RoiItem[];
};

type RawImageTimestamp = {
  sec?: number;
  nsec?: number;
};

type RawImageMessage = {
  timestamp?: RawImageTimestamp;
  width?: number;
  height?: number;
  step?: number;
  encoding?: string;
  data?: unknown;
};

type RawImageThumbnail = {
  frameTimestampNs: string;
  dataUrl: string;
  previewWidth: number;
  previewHeight: number;
};

type RoiRow = {
  key: string;
  frameTimestampNs: string;
  payloadJson: string;
  values: Record<string, number | string>;
};

type LoadStatus = {
  phase: "idle" | "loading" | "ready" | "error" | "unsupported";
  message: string;
};

type ColumnGroup = {
  title: string;
  subcolumns: string[];
};

const DEFAULT_PAGE_SIZE = 100;
const PAGE_SIZE_OPTIONS = [50, 100, 200, 500];
const ROI_TOPIC = "/camera/roi_z_avg";
const RAW_YUYV_TOPIC = "/camera/yuyv";
const RAW_IMAGE_COLUMN_TITLE = "Raw Image";
const EXCEL_IMAGE_COLUMN_WIDTH = 28;
const EXCEL_IMAGE_ROW_HEIGHT = 96;
const RAW_IMAGE_THUMBNAIL_MAX_WIDTH = 180;
const RAW_IMAGE_THUMBNAIL_MAX_HEIGHT = 96;
const ROI_PREFIX_ORDER = [
  "laser_distance_mm",
  "ROI-LT",
  "ROI-MT",
  "ROI-RT",
  "ROI-LM",
  "ROI-CT",
  "ROI-RM",
  "ROI-LD",
  "ROI-MD",
  "ROI-RD",
];

function timeToNanosecondsString(time?: Time): string {
  if (!time) {
    return "";
  }
  return `${BigInt(time.sec) * 1000000000n + BigInt(time.nsec)}`;
}

function rawImageTimestampToNanosecondsString(timestamp?: RawImageTimestamp): string {
  if (timestamp?.sec == undefined || timestamp?.nsec == undefined) {
    return "";
  }
  return `${BigInt(timestamp.sec) * 1000000000n + BigInt(timestamp.nsec)}`;
}

function flattenRoiMessageEvent(event: Immutable<MessageEvent>, index: number): RoiRow {
  const values: Record<string, number | string> = {};
  const message = (event.message as RoiZAvgMessage | undefined) ?? {};
  for (const item of message.items ?? []) {
    const name = item.name?.trim();
    if (!name) {
      continue;
    }
    values[`${name}.z_avg_mm`] =
      item.z_avg_mm == undefined || item.z_avg_mm == null ? "" : item.z_avg_mm;
    if (item.confidence_avg !== undefined) {
      values[`${name}.confidence_avg`] =
        item.confidence_avg == undefined || item.confidence_avg == null ? "" : item.confidence_avg;
    }
  }

  const frameTimestampNs =
    message.frame_timestamp_ns != undefined
      ? String(message.frame_timestamp_ns)
      : timeToNanosecondsString(event.publishTime) || timeToNanosecondsString(event.receiveTime);

  return {
    key: `${frameTimestampNs || timeToNanosecondsString(event.receiveTime)}:${index}`,
    frameTimestampNs,
    payloadJson: JSON.stringify(event.message),
    values,
  };
}

function normalizeBinaryData(data: unknown): Uint8Array | undefined {
  if (data == undefined || data == null) {
    return undefined;
  }
  if (data instanceof Uint8Array) {
    return data;
  }
  if (Array.isArray(data)) {
    return Uint8Array.from(data.filter((value): value is number => typeof value === "number"));
  }
  if (ArrayBuffer.isView(data)) {
    return new Uint8Array(data.buffer, data.byteOffset, data.byteLength);
  }
  if (data instanceof ArrayBuffer) {
    return new Uint8Array(data);
  }
  return undefined;
}

function clampToByte(value: number): number {
  return Math.max(0, Math.min(255, Math.round(value)));
}

function createRawImageThumbnail(event: Immutable<MessageEvent>): RawImageThumbnail | undefined {
  const message = (event.message as RawImageMessage | undefined) ?? {};
  const encoding = message.encoding?.toLowerCase();
  if (encoding !== "yuyv") {
    return undefined;
  }

  const width = message.width ?? 0;
  const height = message.height ?? 0;
  if (width <= 0 || height <= 0) {
    return undefined;
  }

  const rawBytes = normalizeBinaryData(message.data);
  if (!rawBytes) {
    return undefined;
  }

  const step = message.step ?? width * 2;
  const expectedMinSize = step * height;
  if (rawBytes.byteLength < expectedMinSize || typeof document === "undefined") {
    return undefined;
  }

  const canvas = document.createElement("canvas");
  canvas.width = width;
  canvas.height = height;
  const context = canvas.getContext("2d");
  if (!context) {
    return undefined;
  }

  const imageData = context.createImageData(width, height);
  const rgba = imageData.data;

  for (let y = 0; y < height; y += 1) {
    const rowOffset = y * step;
    for (let x = 0; x < width; x += 2) {
      const pixelOffset = rowOffset + x * 2;
      const y0 = rawBytes[pixelOffset] ?? 0;
      const u = rawBytes[pixelOffset + 1] ?? 128;
      const y1 = rawBytes[pixelOffset + 2] ?? y0;
      const v = rawBytes[pixelOffset + 3] ?? 128;

      const c0 = y0 - 16;
      const c1 = y1 - 16;
      const d = u - 128;
      const e = v - 128;

      const r0 = clampToByte((298 * c0 + 409 * e + 128) >> 8);
      const g0 = clampToByte((298 * c0 - 100 * d - 208 * e + 128) >> 8);
      const b0 = clampToByte((298 * c0 + 516 * d + 128) >> 8);
      const rgbaOffset0 = (y * width + x) * 4;
      rgba[rgbaOffset0] = r0;
      rgba[rgbaOffset0 + 1] = g0;
      rgba[rgbaOffset0 + 2] = b0;
      rgba[rgbaOffset0 + 3] = 255;

      if (x + 1 < width) {
        const r1 = clampToByte((298 * c1 + 409 * e + 128) >> 8);
        const g1 = clampToByte((298 * c1 - 100 * d - 208 * e + 128) >> 8);
        const b1 = clampToByte((298 * c1 + 516 * d + 128) >> 8);
        const rgbaOffset1 = (y * width + x + 1) * 4;
        rgba[rgbaOffset1] = r1;
        rgba[rgbaOffset1 + 1] = g1;
        rgba[rgbaOffset1 + 2] = b1;
        rgba[rgbaOffset1 + 3] = 255;
      }
    }
  }

  context.putImageData(imageData, 0, 0);

  const scale = Math.min(
    1,
    RAW_IMAGE_THUMBNAIL_MAX_WIDTH / width,
    RAW_IMAGE_THUMBNAIL_MAX_HEIGHT / height,
  );

  const frameTimestampNs =
    rawImageTimestampToNanosecondsString(message.timestamp) ||
    timeToNanosecondsString(event.publishTime) ||
    timeToNanosecondsString(event.receiveTime);
  if (!frameTimestampNs) {
    return undefined;
  }

  return {
    frameTimestampNs,
    dataUrl: canvas.toDataURL("image/png"),
    previewWidth: Math.max(1, Math.round(width * scale)),
    previewHeight: Math.max(1, Math.round(height * scale)),
  };
}

function sanitizeFilenameSegment(value: string): string {
  return value.replace(/[^a-zA-Z0-9._-]+/g, "_").replace(/^_+|_+$/g, "") || "topic";
}

function inferExportBaseName(): string {
  if (typeof document === "undefined") {
    return sanitizeFilenameSegment(ROI_TOPIC);
  }

  const title = document.title ?? "";
  const mcapMatch = title.match(/([^/\\]+?\.mcap)\b/i);
  if (mcapMatch?.[1]) {
    return sanitizeFilenameSegment(mcapMatch[1].replace(/\.mcap$/i, ""));
  }

  const trimmedTitle = title.trim();
  if (trimmedTitle) {
    return sanitizeFilenameSegment(trimmedTitle);
  }

  return sanitizeFilenameSegment(ROI_TOPIC);
}

function isLaserDistanceGroup(title: string): boolean {
  return title === "laser_distance_mm";
}

function isRoiGroup(title: string): boolean {
  return title.startsWith("ROI-");
}

function makeColumnGroups(columns: string[]): ColumnGroup[] {
  const groups = new Map<string, Set<string>>();

  for (const column of columns) {
    const splitIndex = column.lastIndexOf(".");
    const title = splitIndex >= 0 ? column.slice(0, splitIndex) : column;
    const subcolumn = splitIndex >= 0 ? column.slice(splitIndex + 1) : "value";
    const existing = groups.get(title);
    if (existing) {
      existing.add(subcolumn);
    } else {
      groups.set(title, new Set([subcolumn]));
    }
  }

  for (const [title, subcolumns] of groups.entries()) {
    if (isLaserDistanceGroup(title)) {
      subcolumns.add("z_avg_mm");
      continue;
    }
    if (isRoiGroup(title)) {
      subcolumns.add("z_avg_mm");
      subcolumns.add("confidence_avg");
    }
  }

  return Array.from(groups.entries())
    .sort(([leftTitle], [rightTitle]) => {
      const leftIndex = ROI_PREFIX_ORDER.findIndex((prefix) => leftTitle.startsWith(prefix));
      const rightIndex = ROI_PREFIX_ORDER.findIndex((prefix) => rightTitle.startsWith(prefix));
      if (leftIndex >= 0 || rightIndex >= 0) {
        return (leftIndex >= 0 ? leftIndex : Number.MAX_SAFE_INTEGER) -
          (rightIndex >= 0 ? rightIndex : Number.MAX_SAFE_INTEGER);
      }
      return leftTitle.localeCompare(rightTitle);
    })
    .map(([title, subcolumnSet]) => {
      const subcolumns = Array.from(subcolumnSet).sort((left, right) => {
        const order = ["z_avg_mm", "confidence_avg", "value"];
        const leftIndex = order.indexOf(left);
        const rightIndex = order.indexOf(right);
        if (leftIndex >= 0 || rightIndex >= 0) {
          return (leftIndex >= 0 ? leftIndex : Number.MAX_SAFE_INTEGER) -
            (rightIndex >= 0 ? rightIndex : Number.MAX_SAFE_INTEGER);
        }
        return left.localeCompare(right);
      });
      return { title, subcolumns };
    });
}

async function buildWorkbookBuffer(
  rows: RoiRow[],
  columns: string[],
  rawImageThumbnails: Map<string, RawImageThumbnail>,
  includeRawImages: boolean,
): Promise<ArrayBuffer> {
  const workbook = new ExcelJS.Workbook();
  workbook.creator = "GitHub Copilot";
  workbook.created = new Date();

  const worksheet = workbook.addWorksheet("roi_z_avg", {
    views: [{ state: "frozen", ySplit: 2 }],
  });

  const columnGroups = makeColumnGroups(columns);
  const topHeaderRow = worksheet.getRow(1);
  const subHeaderRow = worksheet.getRow(2);
  topHeaderRow.getCell(1).value = "frame_timestamp_ns";
  worksheet.mergeCells(1, 1, 2, 1);

  let excelColumn = 2;
  for (const group of columnGroups) {
    const groupStart = excelColumn;
    for (const subcolumn of group.subcolumns) {
      subHeaderRow.getCell(excelColumn).value = subcolumn;
      excelColumn += 1;
    }

    if (group.subcolumns.length > 1) {
      worksheet.mergeCells(1, groupStart, 1, excelColumn - 1);
    } else {
      worksheet.mergeCells(1, groupStart, 2, groupStart);
    }
    topHeaderRow.getCell(groupStart).value = group.title;
  }

  let rawImageExcelColumn: number | undefined;
  if (includeRawImages) {
    rawImageExcelColumn = excelColumn;
    topHeaderRow.getCell(rawImageExcelColumn).value = RAW_IMAGE_COLUMN_TITLE;
    worksheet.mergeCells(1, rawImageExcelColumn, 2, rawImageExcelColumn);
  }

  const applyHeaderStyle = (row: ExcelJS.Row) => {
    row.font = { bold: true, color: { argb: "FF0F172A" } };
    row.alignment = { vertical: "middle", horizontal: "center" };
    row.fill = {
      type: "pattern",
      pattern: "solid",
      fgColor: { argb: "FFD8E8FF" },
    };
  };
  applyHeaderStyle(topHeaderRow);
  applyHeaderStyle(subHeaderRow);

  const orderedLeafColumns: string[] = [];
  for (const group of columnGroups) {
    for (const subcolumn of group.subcolumns) {
      orderedLeafColumns.push(`${group.title}.${subcolumn}`);
    }
  }

  for (const row of rows) {
    const worksheetRow = worksheet.addRow([
      row.frameTimestampNs,
      ...orderedLeafColumns.map((column: string) => row.values[column] ?? ""),
      ...(includeRawImages ? [""] : []),
    ]);

    worksheetRow.getCell(1).numFmt = "@";
    worksheetRow.getCell(1).alignment = { horizontal: "left" };
    for (let columnIndex = 0; columnIndex < orderedLeafColumns.length; columnIndex += 1) {
      const columnName = orderedLeafColumns[columnIndex]!;
      const cell = worksheetRow.getCell(columnIndex + 2);
      const value = row.values[columnName] ?? "";
      if (typeof value === "number") {
        cell.value = value;
        cell.numFmt = columnName.endsWith(".confidence_avg") ? "0.00%" : "0.000";
      } else {
        cell.value = value;
      }
    }

    if (includeRawImages && rawImageExcelColumn != undefined) {
      const rawImageCell = worksheetRow.getCell(rawImageExcelColumn);
      rawImageCell.alignment = { horizontal: "center", vertical: "middle" };

      const rawImage = rawImageThumbnails.get(row.frameTimestampNs);
      if (rawImage) {
        worksheetRow.height = EXCEL_IMAGE_ROW_HEIGHT * 0.75;
        const imageId = workbook.addImage({
          base64: rawImage.dataUrl,
          extension: "png",
        });
        worksheet.addImage(imageId, {
          tl: { col: rawImageExcelColumn - 1, row: worksheetRow.number - 1 },
          ext: { width: rawImage.previewWidth, height: rawImage.previewHeight },
        });
      }
    }
  }

  worksheet.columns = [
    { key: "frame_timestamp_ns", width: 24 },
    ...orderedLeafColumns.map((column: string) => ({
      key: column,
      width: column.endsWith(".confidence_avg") ? 10 : column.endsWith(".z_avg_mm") ? 11 : 14,
    })),
    ...(includeRawImages ? [{ key: RAW_IMAGE_COLUMN_TITLE, width: EXCEL_IMAGE_COLUMN_WIDTH }] : []),
  ];

  worksheet.eachRow((row) => {
    row.eachCell((cell) => {
      cell.border = {
        top: { style: "thin", color: { argb: "FFD7E1EE" } },
        left: { style: "thin", color: { argb: "FFD7E1EE" } },
        bottom: { style: "thin", color: { argb: "FFD7E1EE" } },
        right: { style: "thin", color: { argb: "FFD7E1EE" } },
      };
    });
  });

  const buffer = await workbook.xlsx.writeBuffer();
  return buffer as ArrayBuffer;
}

function downloadBinary(filename: string, data: ArrayBuffer, mimeType: string): void {
  const blob = new Blob([data], { type: mimeType });
  const url = URL.createObjectURL(blob);
  const anchor = document.createElement("a");
  anchor.href = url;
  anchor.download = filename;
  anchor.click();
  URL.revokeObjectURL(url);
}

function roiPrefixRank(column: string): number {
  const index = ROI_PREFIX_ORDER.findIndex((prefix) => column.startsWith(prefix));
  return index >= 0 ? index : Number.MAX_SAFE_INTEGER;
}

function orderedColumns(rows: RoiRow[]): string[] {
  const seen = new Set<string>();
  for (const row of rows) {
    for (const key of Object.keys(row.values)) {
      seen.add(key);
    }
  }
  return Array.from(seen).sort((left, right) => {
    const rankDiff = roiPrefixRank(left) - roiPrefixRank(right);
    if (rankDiff !== 0) {
      return rankDiff;
    }
    return left.localeCompare(right);
  });
}

function TopicRangeExportPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const initialState = (context.initialState as PanelState | undefined) ?? {};
  const [colorScheme, setColorScheme] = useState<"dark" | "light">("dark");
  const [pageSize, setPageSize] = useState(initialState.pageSize ?? DEFAULT_PAGE_SIZE);
  const [pageIndex, setPageIndex] = useState(0);
  const [rows, setRows] = useState<RoiRow[]>([]);
  const [columns, setColumns] = useState<string[]>([]);
  const [selectedRowKey, setSelectedRowKey] = useState<string | undefined>();
  const [rawImageCount, setRawImageCount] = useState(0);
  const [includeRawImagesInExport, setIncludeRawImagesInExport] = useState(
    initialState.includeRawImagesInExport ?? true,
  );
  const [status, setStatus] = useState<LoadStatus>({
    phase: "idle",
    message: `Load all messages from ${ROI_TOPIC} in the current MCAP.`,
  });
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();

  const unsubscribeRef = useRef<undefined | (() => void)>();
  const rawImageThumbnailsRef = useRef<Map<string, RawImageThumbnail>>(new Map());
  const loadSequenceRef = useRef(0);

  useLayoutEffect(() => {
    context.onRender = (renderState, done) => {
      setRenderDone(() => done);
      if (renderState.colorScheme) {
        setColorScheme(renderState.colorScheme);
      }
    };

    context.watch("colorScheme");
    context.setDefaultPanelTitle?.("Axera ROI Z Avg Export");

    return () => {
      unsubscribeRef.current?.();
      unsubscribeRef.current = undefined;
    };
  }, [context]);

  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  useEffect(() => {
    context.saveState({ pageSize, includeRawImagesInExport });
  }, [context, pageSize, includeRawImagesInExport]);

  useEffect(() => {
    setPageIndex(0);
  }, [pageSize]);

  useEffect(() => {
    unsubscribeRef.current?.();
    unsubscribeRef.current = undefined;
    loadSequenceRef.current += 1;
    const loadSequence = loadSequenceRef.current;

    setRows([]);
    setColumns([]);
    setRawImageCount(0);
    setSelectedRowKey(undefined);
    rawImageThumbnailsRef.current = new Map();

    if (!context.subscribeMessageRange) {
      setStatus({
        phase: "unsupported",
        message:
          "The current Foxglove data source does not support full-range topic loading. Open an MCAP in Foxglove and use this panel there.",
      });
      return;
    }

    const subscribeMessageRange = context.subscribeMessageRange;
    setStatus({ phase: "loading", message: `Loading all messages from ${ROI_TOPIC}...` });

    const loadAllMessages = async (): Promise<void> => {
      const nextRows: RoiRow[] = [];
      const roiTimestamps = new Set<string>();

      await new Promise<void>((resolve) => {
        unsubscribeRef.current = subscribeMessageRange({
          topic: ROI_TOPIC,
          onNewRangeIterator: async (batchIterator) => {
            let totalMessages = 0;
            let batchIndex = 0;

            for await (const batch of batchIterator) {
              if (loadSequenceRef.current !== loadSequence) {
                resolve();
                return;
              }

              batch.forEach((event, messageIndex) => {
                const row = flattenRoiMessageEvent(event, batchIndex * 1000000 + messageIndex);
                nextRows.push(row);
                if (row.frameTimestampNs) {
                  roiTimestamps.add(row.frameTimestampNs);
                }
              });
              totalMessages += batch.length;
              batchIndex += 1;

              setRows([...nextRows]);
              setColumns(orderedColumns(nextRows));
              setStatus({
                phase: "loading",
                message: `Loading ${ROI_TOPIC}: ${totalMessages} messages received...`,
              });
            }

            resolve();
          },
        });
      });

      if (loadSequenceRef.current !== loadSequence) {
        return;
      }

      const nextColumns = orderedColumns(nextRows);
      setRows(nextRows);
      setColumns(nextColumns);
      setSelectedRowKey(nextRows[0]?.key);

      setStatus({
        phase: "loading",
        message: `Loading ${RAW_YUYV_TOPIC}: matching images for ${nextRows.length} ROI messages...`,
      });

      const rawImageThumbnails = new Map<string, RawImageThumbnail>();
      await new Promise<void>((resolve) => {
        unsubscribeRef.current = subscribeMessageRange({
          topic: RAW_YUYV_TOPIC,
          onNewRangeIterator: async (batchIterator) => {
            let matchedImages = 0;

            for await (const batch of batchIterator) {
              if (loadSequenceRef.current !== loadSequence) {
                resolve();
                return;
              }

              batch.forEach((event) => {
                const thumbnail = createRawImageThumbnail(event);
                if (!thumbnail || !roiTimestamps.has(thumbnail.frameTimestampNs)) {
                  return;
                }
                rawImageThumbnails.set(thumbnail.frameTimestampNs, thumbnail);
              });

              matchedImages = rawImageThumbnails.size;
              setRawImageCount(matchedImages);
              setStatus({
                phase: "loading",
                message: `Loading ${RAW_YUYV_TOPIC}: matched ${matchedImages}/${nextRows.length} images...`,
              });
            }

            resolve();
          },
        });
      });

      if (loadSequenceRef.current !== loadSequence) {
        return;
      }

      rawImageThumbnailsRef.current = rawImageThumbnails;
      setRawImageCount(rawImageThumbnails.size);
      setStatus({
        phase: "ready",
        message: `Loaded ${nextRows.length} messages from ${ROI_TOPIC} and matched ${rawImageThumbnails.size} raw images.`,
      });
    };

    void loadAllMessages().catch((error: unknown) => {
      if (loadSequenceRef.current !== loadSequence) {
        return;
      }
      setStatus({
        phase: "error",
        message: error instanceof Error ? error.message : "Failed to load topic messages.",
      });
    });

    return () => {
      if (loadSequenceRef.current === loadSequence) {
        unsubscribeRef.current?.();
        unsubscribeRef.current = undefined;
      }
    };
  }, [context]);

  const selectedRow = rows.find((row) => row.key === selectedRowKey) ?? rows[0];
  const totalPages = Math.max(1, Math.ceil(rows.length / pageSize));
  const safePageIndex = Math.min(pageIndex, totalPages - 1);
  const pageStart = safePageIndex * pageSize;
  const visibleRows = rows.slice(pageStart, pageStart + pageSize);

  const isDark = colorScheme === "dark";
  const palette = {
    background: isDark ? "#0e1623" : "#f4f7fb",
    panel: isDark ? "#162133" : "#ffffff",
    panelAlt: isDark ? "#101927" : "#edf2f8",
    text: isDark ? "#ecf3ff" : "#142033",
    muted: isDark ? "#92a0b8" : "#5d6c83",
    border: isDark ? "#25344e" : "#d7e1ee",
    accent: "#2e9bff",
    accentText: "#ffffff",
    danger: "#d96b6b",
  };

  return (
    <div
      style={{
        display: "grid",
        gridTemplateRows: "auto auto minmax(0, 1fr) minmax(0, 220px)",
        gap: 12,
        height: "100%",
        padding: 12,
        boxSizing: "border-box",
        background: `linear-gradient(180deg, ${palette.background} 0%, ${palette.panelAlt} 100%)`,
        color: palette.text,
        fontFamily: '"Segoe UI", "Helvetica Neue", sans-serif',
      }}>
      <div
        style={{
          display: "flex",
          justifyContent: "space-between",
          alignItems: "center",
          gap: 12,
          padding: 12,
          borderRadius: 12,
          background: palette.panel,
          border: `1px solid ${palette.border}`,
        }}>
        <div>
          <div style={{ fontSize: 18, fontWeight: 700 }}>Axera ROI Z Avg Export</div>
          <div style={{ color: palette.muted, marginTop: 4 }}>{status.message}</div>
        </div>
        <div style={{ display: "flex", alignItems: "center", gap: 12, flexWrap: "wrap" }}>
          <label
            style={{
              display: "inline-flex",
              alignItems: "center",
              gap: 8,
              color: palette.muted,
              fontSize: 13,
              userSelect: "none",
            }}>
            <input
              type="checkbox"
              checked={includeRawImagesInExport}
              onChange={(event) => setIncludeRawImagesInExport(event.target.checked)}
            />
            Export YUYV Image
          </label>
          <button
            onClick={async () => {
              if (rows.length === 0) {
                return;
              }
              const filename = `${inferExportBaseName()}_${rows.length}_frames.xlsx`;
              const workbookBuffer = await buildWorkbookBuffer(
                rows,
                columns,
                rawImageThumbnailsRef.current,
                includeRawImagesInExport,
              );
              downloadBinary(
                filename,
                workbookBuffer,
                "application/vnd.openxmlformats-officedocument.spreadsheetml.sheet",
              );
            }}
            disabled={rows.length === 0}
            style={{
              border: 0,
              borderRadius: 10,
              padding: "10px 16px",
              background: rows.length > 0 ? palette.accent : palette.border,
              color: rows.length > 0 ? palette.accentText : palette.muted,
              cursor: rows.length > 0 ? "pointer" : "not-allowed",
              fontWeight: 700,
            }}>
            Export Excel
          </button>
        </div>
      </div>

      <div
        style={{
          display: "grid",
          gridTemplateColumns: "minmax(240px, 2fr) 140px auto",
          gap: 12,
          padding: 12,
          borderRadius: 12,
          background: palette.panel,
          border: `1px solid ${palette.border}`,
          alignItems: "end",
        }}>
        <div style={{ display: "grid", gap: 6 }}>
          <span style={{ fontSize: 12, color: palette.muted, textTransform: "uppercase" }}>
            Topic
          </span>
          <div
            style={{
              width: "100%",
              padding: "10px 12px",
              borderRadius: 10,
              border: `1px solid ${palette.border}`,
              background: palette.panelAlt,
              color: palette.text,
              boxSizing: "border-box",
            }}>
            {ROI_TOPIC}
          </div>
        </div>

        <label style={{ display: "grid", gap: 6 }}>
          <span style={{ fontSize: 12, color: palette.muted, textTransform: "uppercase" }}>
            Page Size
          </span>
          <select
            value={pageSize}
            onChange={(event) => setPageSize(Number(event.target.value))}
            style={{
              width: "100%",
              padding: "10px 12px",
              borderRadius: 10,
              border: `1px solid ${palette.border}`,
              background: palette.panelAlt,
              color: palette.text,
            }}>
            {PAGE_SIZE_OPTIONS.map((option) => (
              <option key={option} value={option}>
                {option}
              </option>
            ))}
          </select>
        </label>

        <div style={{ display: "flex", gap: 16, flexWrap: "wrap", color: palette.muted }}>
          <span>Messages: {rows.length}</span>
          <span>ROI Columns: {columns.length}</span>
          <span>Raw Images: {rawImageCount}</span>
          <span>Page: {safePageIndex + 1}/{totalPages}</span>
        </div>
      </div>

      <div
        style={{
          minHeight: 0,
          display: "grid",
          gridTemplateRows: "auto minmax(0, 1fr) auto",
          borderRadius: 12,
          background: palette.panel,
          border: `1px solid ${palette.border}`,
          overflow: "hidden",
        }}>
        <div
          style={{
            display: "flex",
            justifyContent: "space-between",
            alignItems: "center",
            padding: 12,
            borderBottom: `1px solid ${palette.border}`,
          }}>
          <div style={{ fontWeight: 700 }}>Messages</div>
          <div style={{ display: "flex", gap: 8 }}>
            <button
              onClick={() => setPageIndex((current) => Math.max(0, current - 1))}
              disabled={safePageIndex === 0}
              style={{
                borderRadius: 8,
                border: `1px solid ${palette.border}`,
                background: palette.panelAlt,
                color: palette.text,
                padding: "6px 10px",
                cursor: safePageIndex === 0 ? "not-allowed" : "pointer",
              }}>
              Prev
            </button>
            <button
              onClick={() => setPageIndex((current) => Math.min(totalPages - 1, current + 1))}
              disabled={safePageIndex >= totalPages - 1}
              style={{
                borderRadius: 8,
                border: `1px solid ${palette.border}`,
                background: palette.panelAlt,
                color: palette.text,
                padding: "6px 10px",
                cursor: safePageIndex >= totalPages - 1 ? "not-allowed" : "pointer",
              }}>
              Next
            </button>
          </div>
        </div>

        <div style={{ overflow: "auto" }}>
          <table style={{ width: "100%", borderCollapse: "collapse", fontSize: 12 }}>
            <thead style={{ position: "sticky", top: 0, background: palette.panel }}>
              <tr>
                <th
                  key="frame_timestamp_ns"
                  style={{
                    textAlign: "left",
                    padding: "10px 12px",
                    borderBottom: `1px solid ${palette.border}`,
                    whiteSpace: "nowrap",
                  }}>
                  frame_timestamp_ns
                </th>
                {columns.map((column) => (
                  <th
                    key={column}
                    style={{
                      textAlign: "left",
                      padding: "10px 12px",
                      borderBottom: `1px solid ${palette.border}`,
                      whiteSpace: "nowrap",
                    }}>
                    {column}
                  </th>
                ))}
              </tr>
            </thead>
            <tbody>
              {visibleRows.map((row) => {
                const isSelected = row.key === selectedRow?.key;
                return (
                  <tr
                    key={row.key}
                    onClick={() => setSelectedRowKey(row.key)}
                    style={{
                      background: isSelected ? palette.panelAlt : "transparent",
                      cursor: "pointer",
                    }}>
                    <td style={{ padding: "8px 12px", borderBottom: `1px solid ${palette.border}` }}>
                      {row.frameTimestampNs}
                    </td>
                    {columns.map((column) => (
                      <td
                        key={`${row.key}:${column}`}
                        style={{
                          padding: "8px 12px",
                          borderBottom: `1px solid ${palette.border}`,
                          maxWidth: 280,
                          whiteSpace: "nowrap",
                          overflow: "hidden",
                          textOverflow: "ellipsis",
                        }}>
                        {row.values[column] ?? ""}
                      </td>
                    ))}
                  </tr>
                );
              })}
            </tbody>
          </table>
        </div>

        <div
          style={{
            display: "flex",
            justifyContent: "space-between",
            alignItems: "center",
            padding: 12,
            borderTop: `1px solid ${palette.border}`,
            color: palette.muted,
          }}>
          <span>
            Showing {rows.length === 0 ? 0 : pageStart + 1}-{Math.min(pageStart + pageSize, rows.length)} of {rows.length}
          </span>
          {status.phase === "error" ? <span style={{ color: palette.danger }}>{status.message}</span> : null}
        </div>
      </div>

      <div
        style={{
          minHeight: 0,
          borderRadius: 12,
          background: palette.panel,
          border: `1px solid ${palette.border}`,
          overflow: "hidden",
          display: "grid",
          gridTemplateRows: "auto minmax(0, 1fr)",
        }}>
        <div style={{ padding: 12, borderBottom: `1px solid ${palette.border}`, fontWeight: 700 }}>
          Selected ROI JSON
        </div>
        <pre
          style={{
            margin: 0,
            padding: 12,
            overflow: "auto",
            fontSize: 12,
            lineHeight: 1.5,
            color: palette.text,
          }}>
          {selectedRow ? JSON.stringify(JSON.parse(selectedRow.payloadJson), null, 2) : "No message selected."}
        </pre>
      </div>
    </div>
  );
}

export function initTopicRangeExportPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<TopicRangeExportPanel context={context} />);
  return () => {
    root.unmount();
  };
}
