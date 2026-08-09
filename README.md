# EvergreenSLAM

EvergreenSLAM is a lightweight, always-on 2D LiDAR SLAM framework for embodied agents and DIY robots.

Instead of separating mapping and localization, it continuously builds, localizes, freezes well-constrained submaps, and expands into unknown areas when needed.

Its design philosophy is simple:

- **Always-on**
- **Zero-touch**
- **Freeze only when stable**
- **Grow from trusted maps**
- **Simple by design**
- **Agent-ready**

The goal is to make SLAM a background spatial service rather than a manually operated mapping tool.
