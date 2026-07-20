# Media assets · 媒体素材

Drop screenshots, videos, and animations here. The root [`README.md`](../../README.md)
already has commented Markdown slots — uncomment them once files exist.

## Recommended set · 建议清单

| Filename | Type | Spec (suggested) | Purpose |
|----------|------|------------------|---------|
| `banner.png` | image | ~1600×480, dark/clean | GitHub header |
| `demo.mp4` | video | 30–90 s, 1080p, H.264 | Overview reel |
| `demo-poster.png` | image | 16:9 cover frame | Clickable video poster |
| `hero-dense-field.gif` | gif | ≤8 MB, 5–15 s loop | Hero animation |
| `architecture.png` | diagram | plant ↔ planners | System architecture |
| `console-ui.png` | screenshot | Mission Console | Product UI |
| `console-ui.gif` | gif | optional walkthrough | UI motion |
| `dense-field.gif` / `.mp4` | clip | Path G/H dense | Autonomy demo |
| `swarm.gif` / `.mp4` | clip | swarm / formation | Multi-UAV |
| `rviz-overview.png` | screenshot | canonical RViz | Viz layout |
| `sac-training.gif` | gif | train card / curves | Learning feature |

## Naming · 命名

Prefer lowercase kebab-case. Keep GitHub-friendly sizes:

- GIF / PNG under ~10 MB when possible  
- Prefer linking large MP4 from Releases or an external host if the repo grows

## How to show in README · 如何显示

1. Add the file under `docs/media/`.
2. In root `README.md`, find the matching HTML placeholder or commented `![...](...)`.
3. Remove the placeholder line and uncomment the Markdown image/video link.

Example:

```markdown
![Dense-field flight](docs/media/hero-dense-field.gif)
```

## Capture tips · 拍摄提示

- **RViz**: use `drone.rviz`; show yellow planned path + blue flown path.  
- **Console**: single Path H + dense map, or multi formation page.  
- **Training**: Path H train card with success / steps visible.  
- Export GIF via Peek / `ffmpeg` from a short screen recording.
