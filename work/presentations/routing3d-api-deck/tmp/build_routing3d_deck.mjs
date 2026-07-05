import fs from "node:fs/promises";
import path from "node:path";
import { Presentation, PresentationFile } from "@oai/artifact-tool";

const INPUT_MD = "D:/DINNO/DEV/AI-AutoRouting/Routing3D/cpp/docs/Routing3D_CPP_Engine_API_Presentation.md";
const FINAL_PPTX = "D:/DINNO/DEV/AI-AutoRouting/Routing3D/cpp/docs/Routing3D_CPP_Engine_API_Presentation.pptx";
const OUT_DIR = "D:/DINNO/DEV/AI-AutoRouting/Routing3D/work/presentations/routing3d-api-deck/tmp/qa";
const W = 1280;
const H = 720;
const C = {
  bg: "#0f172a",
  panel: "#1e293b",
  panel2: "#111827",
  accent: "#00f0ff",
  code: "#ff007f",
  text: "#f8fafc",
  muted: "#94a3b8",
  line: "#475569",
};

function stripFrontMatter(src) {
  return src.replace(/^---\r?\n[\s\S]*?\r?\n---\r?\n/, "");
}

function splitSlides(src) {
  return stripFrontMatter(src)
    .split(/\r?\n---\r?\n/g)
    .map((s) => s.trim())
    .filter(Boolean);
}

function parseSlide(raw, idx) {
  const lines = raw.split(/\r?\n/);
  let title = "";
  const rest = [];
  for (const line of lines) {
    const h = line.match(/^(#{1,3})\s+(.*)$/);
    if (!title && h) {
      title = h[2].trim();
    } else {
      rest.push(line);
    }
  }
  if (!title) title = `Slide ${idx + 1}`;
  return { title, blocks: parseBlocks(rest.join("\n").trim()) };
}

function parseBlocks(text) {
  const lines = text.split(/\r?\n/);
  const blocks = [];
  let buffer = [];
  let code = null;

  const flushText = () => {
    const value = buffer.join("\n").trim();
    if (value) blocks.push({ type: "text", text: value });
    buffer = [];
  };

  for (const line of lines) {
    const fence = line.match(/^```(\w+)?\s*$/);
    if (fence && !code) {
      flushText();
      code = { lang: fence[1] || "", lines: [] };
      continue;
    }
    if (fence && code) {
      blocks.push({ type: "code", lang: code.lang, text: code.lines.join("\n") });
      code = null;
      continue;
    }
    if (code) {
      code.lines.push(line);
      continue;
    }
    buffer.push(line);
  }
  flushText();

  const normalized = [];
  for (const block of blocks) {
    if (block.type !== "text") {
      normalized.push(block);
      continue;
    }
    const bLines = block.text.split(/\r?\n/);
    let current = [];
    const pushCurrent = () => {
      const t = current.join("\n").trim();
      if (t) normalized.push({ type: "text", text: t });
      current = [];
    };
    let table = [];
    const pushTable = () => {
      if (table.length) normalized.push({ type: "tableText", text: table.join("\n") });
      table = [];
    };
    for (const line of bLines) {
      if (/^\s*\|.*\|\s*$/.test(line)) {
        pushCurrent();
        table.push(line);
      } else {
        pushTable();
        current.push(line);
      }
    }
    pushTable();
    pushCurrent();
  }
  return normalized;
}

function cleanMarkdown(text) {
  return text
    .replace(/^#{1,6}\s+/gm, "")
    .replace(/\*\*([^*]+)\*\*/g, "$1")
    .replace(/\*\*/g, "")
    .replace(/`([^`]+)`/g, "$1")
    .replace(/\$\$/g, "")
    .replace(/\\_/g, "_")
    .replace(/\\\*/g, "*")
    .replace(/\s+$/gm, "");
}

function fontForLines(lineCount, longest, kind) {
  if (kind === "code") {
    if (lineCount > 16 || longest > 92) return 12;
    if (lineCount > 11 || longest > 76) return 14;
    return 16;
  }
  if (kind === "table") {
    if (lineCount > 9 || longest > 105) return 10.5;
    if (longest > 86) return 12;
    return 14;
  }
  if (lineCount > 19 || longest > 80) return 15;
  if (lineCount > 14) return 16;
  return 18;
}

function addBox(slide, x, y, w, h, fill = "none", line = "none") {
  return slide.shapes.add({
    geometry: "rect",
    position: { left: x, top: y, width: w, height: h },
    fill,
    line: { style: "solid", fill: line, width: line === "none" ? 0 : 1 },
  });
}

function addText(slide, text, x, y, w, h, opts = {}) {
  const shape = slide.shapes.add({
    geometry: "textbox",
    position: { left: x, top: y, width: w, height: h },
    fill: opts.fill ?? "none",
    line: { style: "solid", fill: opts.line ?? "none", width: opts.line ? 1 : 0 },
  });
  shape.text = text;
  shape.text.style = {
    fontSize: opts.fontSize ?? 18,
    bold: opts.bold ?? false,
    color: opts.color ?? C.text,
    typeface: opts.typeface ?? "Segoe UI",
    alignment: opts.alignment ?? "left",
  };
  return shape;
}

function slideChrome(slide, title, n, total) {
  slide.background.fill = C.bg;
  addBox(slide, 0, 0, W, 720, C.bg, "none");
  addBox(slide, 40, 104, 1200, 2, "none", "#155e75");
  addText(slide, title, 44, 34, 1058, 58, {
    fontSize: title.length > 42 ? 31 : 35,
    bold: true,
    color: C.accent,
    typeface: "Segoe UI Semibold",
  });
  addText(slide, `Routing3D C++ Engine API  |  ${n}/${total}`, 44, 676, 650, 22, {
    fontSize: 12,
    color: C.muted,
  });
  addText(slide, `Slide ${n}`, 1120, 675, 112, 24, {
    fontSize: 13,
    color: C.muted,
    alignment: "right",
  });
}

function layoutBlocks(slide, blocks) {
  let y = 126;
  const x = 62;
  const w = 1156;
  const gap = 12;
  const availableBottom = 654;
  const remaining = blocks.length || 1;
  for (let i = 0; i < blocks.length; i++) {
    const block = blocks[i];
    const lines = block.text.split(/\r?\n/);
    const longest = Math.max(...lines.map((l) => l.length), 1);
    const rest = blocks.length - i - 1;
    const maxH = availableBottom - y - rest * 82;
    if (block.type === "code" || block.type === "tableText") {
      const kind = block.type === "code" ? "code" : "table";
      const fontSize = fontForLines(lines.length, longest, kind);
      const h = Math.max(76, Math.min(maxH, lines.length * (fontSize + 6) + 30));
      const fill = block.type === "code" ? C.panel : "#172033";
      addBox(slide, x, y, w, h, fill, C.line);
      addText(slide, cleanMarkdown(block.text), x + 18, y + 14, w - 36, h - 24, {
        fontSize,
        color: block.type === "code" ? "#fbcfe8" : C.text,
        typeface: "Cascadia Mono",
      });
      y += h + gap;
    } else {
      const text = cleanMarkdown(block.text);
      const fontSize = fontForLines(lines.length, longest, "text");
      const h = Math.max(58, Math.min(maxH, lines.length * (fontSize + 8) + 10));
      addText(slide, text, x, y, w, h, {
        fontSize,
        color: C.text,
        typeface: "Segoe UI",
      });
      y += h + gap;
    }
  }
}

function addTitleSlide(slide, parsed, total) {
  slide.background.fill = C.bg;
  addBox(slide, 0, 0, W, H, C.bg, "none");
  addBox(slide, 64, 92, 5, 380, C.accent, "none");
  addText(slide, parsed.title, 92, 104, 940, 88, {
    fontSize: 50,
    bold: true,
    color: C.accent,
    typeface: "Segoe UI Semibold",
  });
  const body = cleanMarkdown(parsed.blocks.map((b) => b.text).join("\n\n"));
  addText(slide, body, 96, 220, 760, 220, {
    fontSize: 23,
    color: C.text,
  });
  addBox(slide, 900, 160, 246, 246, C.panel, C.line);
  addText(slide, "C++\nAPI", 940, 218, 170, 120, {
    fontSize: 48,
    bold: true,
    color: C.code,
    alignment: "center",
    typeface: "Cascadia Mono",
  });
  addText(slide, `1/${total}`, 1092, 666, 110, 28, { fontSize: 14, color: C.muted, alignment: "right" });
}

function addDiagramSlide(slide, parsed, n, total) {
  slideChrome(slide, parsed.title, n, total);
  const nodes = [
    ["C ABI\nrouting3d_capi", 92, 145],
    ["CLI\nrouting3d_cli", 92, 285],
    ["SceneDoc\n공통 데이터 모델", 380, 215],
    ["SceneIO\nscene.json v3", 380, 355],
    ["Occupancy 백엔드\nDense / Implicit / Octree / VDB", 675, 145],
    ["CostModel\n회전 / 이격 / 랙고도", 675, 285],
    ["알고리즘 계층\nWeighted A* / Segment A* / HPA*", 675, 425],
    ["Result 계층\nAStarResult / R3dResult", 970, 285],
  ];
  for (const [label, x, y] of nodes) {
    addBox(slide, x, y, 218, 76, C.panel, C.line);
    addText(slide, label, x + 12, y + 12, 194, 48, {
      fontSize: 18,
      bold: /C ABI|SceneDoc|Occupancy|알고리즘|Result/.test(label),
      color: C.text,
      alignment: "center",
    });
  }
  const arrow = (x1, y1, x2, y2) => {
    const line = slide.shapes.add({
      geometry: "line",
      position: { left: x1, top: y1, width: x2 - x1, height: y2 - y1 },
      line: { style: "solid", fill: C.accent, width: 2, endArrowType: "triangle" },
    });
    return line;
  };
  arrow(310, 183, 380, 244);
  arrow(310, 323, 380, 392);
  arrow(598, 253, 675, 183);
  arrow(598, 253, 675, 323);
  arrow(784, 221, 784, 285);
  arrow(784, 361, 784, 425);
  arrow(893, 463, 970, 323);
}

async function writeBlob(filePath, blob) {
  await fs.mkdir(path.dirname(filePath), { recursive: true });
  await fs.writeFile(filePath, Buffer.from(await blob.arrayBuffer()));
}

async function main() {
  await fs.mkdir(OUT_DIR, { recursive: true });
  const md = await fs.readFile(INPUT_MD, "utf8");
  const parsed = splitSlides(md).map(parseSlide);
  const presentation = Presentation.create({ slideSize: { width: W, height: H } });
  const total = parsed.length;
  parsed.forEach((p, idx) => {
    const slide = presentation.slides.add();
    if (idx === 0) addTitleSlide(slide, p, total);
    else if (idx === 3) addDiagramSlide(slide, p, idx + 1, total);
    else {
      slideChrome(slide, p.title, idx + 1, total);
      layoutBlocks(slide, p.blocks);
    }
  });

  for (const [idx, slide] of presentation.slides.items.entries()) {
    const stem = `slide-${String(idx + 1).padStart(2, "0")}`;
    await writeBlob(path.join(OUT_DIR, `${stem}.png`), await presentation.export({ slide, format: "png", scale: 1 }));
    await fs.writeFile(path.join(OUT_DIR, `${stem}.layout.json`), await (await slide.export({ format: "layout" })).text(), "utf8");
  }
  await writeBlob(path.join(OUT_DIR, "deck-montage.webp"), await presentation.export({ format: "webp", montage: true, scale: 1 }));
  const snapshot = await presentation.inspect({ kind: "slide,textbox,shape,layout", maxChars: 12000 });
  await fs.writeFile(path.join(OUT_DIR, "inspect.ndjson"), snapshot.ndjson, "utf8");
  const pptx = await PresentationFile.exportPptx(presentation);
  await pptx.save(FINAL_PPTX);
  console.log(`slides=${total}`);
  console.log(FINAL_PPTX);
}

main().catch((error) => {
  console.error(error);
  process.exitCode = 1;
});
