#!/usr/bin/env python3
"""Wrapper: run waypoint_publisher from drone_bringup package."""
from drone_bringup.waypoint_publisher import main

if __name__ == '__main__':
    raise SystemExit(main())
