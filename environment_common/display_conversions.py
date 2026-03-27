import yaml
from graphviz import Digraph
from pathlib import Path
from collections import defaultdict


def generate_sources_png(et_src):
    # === CONFIG ===
    base_path = Path(et_src).expanduser().resolve()
    config_dir = base_path / "config"
    log_file = config_dir / "conversion_sources.yaml"
    output_file = config_dir / "conversion_sources_overlay"
    include_exts = {".yaml", ".kml", ".png", ".world"}

    # --- Load conversion log ---
    if not log_file.exists():
        raise FileNotFoundError(f"❌ Could not find {log_file}")

    with open(log_file) as f:
        data = yaml.safe_load(f).get("conversion_sources", {})

    # --- Collect dependency relationships ---
    targets = set(data.keys())
    sources = set()
    for info in data.values():
        for src in info.get("data_sources", []):
            sources.add(src["path"])

    # --- Classify files ---
    input_only = sources - targets
    input_and_output = targets & sources
    output_only = targets - sources

    # --- Collect all relevant files (under subdirectories only) ---
    all_files = {
        str(p.relative_to(config_dir))
        for p in config_dir.rglob("*")
        if p.is_file()
        and p.suffix in include_exts
        and len(p.relative_to(config_dir).parts) > 1
    }

    unused = all_files - (targets | sources)

    # --- Group by first subdirectory ---
    grouped = defaultdict(list)
    for path in all_files:
        parts = Path(path).parts
        group = parts[0]
        grouped[group].append(path)

    # --- Create Graph ---
    dot = Digraph(comment="Map Conversion Dependencies", format="png")
    dot.engine = "dot"  # stable, clean cluster layout

    # --- Global layout & quality ---
    dot.attr(
        bgcolor="white",
        rankdir="TB",
        nodesep="0.35",
        ranksep="0.6",
        overlap="false",
        splines="true",
        margin="0.15",
        pack="true",
        packmode="cluster",
        fontname="Helvetica",
    )
    dot.attr("graph", dpi="160", pad="0.05")
    dot.attr("node", fontname="Helvetica", fontsize="9")

    # --- Colors ---
    COLOR_OUTPUT = "#fff97e"
    COLOR_INPUT = "#7effa3"
    COLOR_UNUSED = "#d0d0d0"
    BORDER_OUTPUT = "#b8a800"
    BORDER_INPUT = "#2e7d32"
    BORDER_UNUSED = "#555555"

    def fill_and_border_for(path):
        if path in output_only or path in input_and_output:
            return COLOR_OUTPUT, BORDER_OUTPUT
        if path in input_only:
            return COLOR_INPUT, BORDER_INPUT
        if path in unused:
            return COLOR_UNUSED, BORDER_UNUSED
        return "#ffffff", "#555555"

    # --- Create subdirectory clusters ---
    for group, files in sorted(grouped.items()):
        with dot.subgraph(name=f"cluster_{group}") as sg:
            sg.attr(
                label=group,
                style="rounded,filled",
                color="#dddddd",
                fillcolor="#f9f9f9",
                fontname="Helvetica-Bold",
                fontsize="11",
                margin="10"
            )

            for path in sorted(files):
                full_path = (config_dir / path).as_posix()
                fill, border = fill_and_border_for(path)

                # --- Smaller PNG thumbnails ---
                if path.endswith(".png") and Path(full_path).exists():
                    sg.node(
                        path,
                        label=Path(path).name,
                        image=full_path,
                        shape="box",
                        style="filled",
                        fillcolor=fill,
                        color=border,
                        penwidth="1.2",
                        imagescale="true",
                        labelloc="b",
                        fixedsize="true",      # enforce consistent size
                        width="0.9", height="0.9",  # smaller thumbnails
                        margin="0.08,0.05"     # padding between image and text
                    )
                else:
                    # --- Regular nodes ---
                    shape = "box" if (path in output_only or path in input_and_output) else "ellipse"
                    sg.node(
                        path,
                        label=path,
                        style="filled",
                        fillcolor=fill,
                        color=border,
                        penwidth="1",
                        shape=shape,
                        margin="0.15",
                        width="0",
                        height="0"
                    )

    # --- Add dependency edges ---
    for target, info in data.items():
        for src in info.get("data_sources", []):
            dot.edge(src["path"], target)

    # --- Render graph ---
    output_path = dot.render(output_file, cleanup=False)
    print(f"✅ Saved dependency overlay to: {output_path}")

    # --- Summary ---
    print("\n📄 Files by category:")
    print(f"  🟨 Output (only / reused): {len(output_only | input_and_output)}")
    print(f"  🟩 Input only: {len(input_only)}")
    print(f"  ⚫ Unused: {len(unused)}\n")


# === Example manual test ===
if __name__ == "__main__":
    generate_sources_png("~/ros2_ws/src/environment_template")
