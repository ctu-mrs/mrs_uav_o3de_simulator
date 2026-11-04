# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

Drone simulation built on the O3DE (Open 3D Engine) game engine.

## Current state

The repository holds only a README, this file, and `.gitignore`. There is no O3DE project, Gem, or build setup yet, so there are no build, lint, or test commands to document. Update this file once the project structure and CMake configuration are in place.

## Repository conventions

- Default branch: `main` (remote `origin` → `git@github.com:klaxalk/o3de_drone_sim.git`).
- `.gitignore` excludes O3DE's generated directories (`build/`, `Cache/`, `user/`, `_savebackup/`). Do not commit their contents.
