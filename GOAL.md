# Workspace Status System Refactoring

## Objective

Refactor the workspace status management system to have clear separation of concerns, with each component having a single responsibility.

## Current Issues

1. Multiple scripts directly manipulate workspace-status.json (claude-workspace-status, long command)
2. i3-voyager-rgb handles too many responsibilities (i3 monitoring, state management, LED control)
3. Naming is confusing (claude-workspace-status handles more than just Claude states)
4. State priority logic is scattered across multiple scripts

## New Architecture

```
┌─────────────┐     ┌──────────────┐      ┌──────────────────┐
│   i3 WM     │────▶│ i3-workspace │───-─▶│                  │
│   events    │     │   -monitor   │      │                  │
└─────────────┘     └──────────────┘      │                  │
                                          │                  │
┌─────────────┐                           │  set-workspace   │
│Claude hooks │──────────────────────────▶|     -status      │
└─────────────┘                           │                  │
                                          │                  │
┌─────────────┐                           │                  │
│long command │──────────────────────────▶|                  │
└─────────────┘                           └────────┬─────────┘
                                                   │
┌──────────────────┐                               ▼
│ voyager          │◀──────────────────── workspace-status.json
│ -workspace       │
│ -status-daemon   │
└──────────────────┘
```

## Components

### 1. `set-workspace-status` (NEW)

**Purpose**: Single API for all workspace state changes
**Responsibilities**:

- ONLY writer to workspace-status.json
- Handle state priority and conflicts
- Manage state subsystems (ai-_, task-_, etc.)

**Usage Examples**:

```bash
# AI states (mutually exclusive within AI subsystem)
set-workspace-status ai-ready      # Clear other ai-*, set ai-ready
set-workspace-status ai-busy       # Clear other ai-*, set ai-busy
set-workspace-status ai-complete   # Special: sets ai-ready AND ai-complete
set-workspace-status ai-exited     # Clear ALL ai-* states

# Task states (mutually exclusive within task subsystem)
set-workspace-status task-running   # Clear other task-*, set task-running
set-workspace-status task-succeeded # Clear other task-*, set task-succeeded
set-workspace-status task-failed    # Clear other task-*, set task-failed
set-workspace-status task-clear     # Clear ALL task-* states

# i3 states (managed by i3-workspace-monitor)
set-workspace-status focused       # Add focused state
set-workspace-status unfocused     # Remove focused state
set-workspace-status urgent        # Add urgent state
set-workspace-status occupied      # Set occupied
set-workspace-status empty         # Set empty

```

### 2. `i3-workspace-monitor` (EXTRACTED FROM i3-voyager-rgb)

**Purpose**: Monitor i3 window manager events
**Responsibilities**:

- Subscribe to i3 events (workspace, window)
- Call set-workspace-status for:
  - Focus changes (focused/unfocused)
  - Window presence (occupied/empty)
  - Urgent state changes

### 3. `voyager-workspace-status-daemon` (RENAMED FROM i3-voyager-rgb)

**Purpose**: Display workspace status on Voyager keyboard LEDs
**Responsibilities**:

- Watch workspace-status.json for changes
- Map states to LED colors using priority
- Update Voyager keyboard RGB via Keymapp API
- NO state management logic

### 4. Updates to Existing Scripts

#### `long` command

Replace direct JSON manipulation with:

```bash
set-workspace-status task-running    # When starting
set-workspace-status task-succeeded  # On success
set-workspace-status task-failed     # On failure
set-workspace-status task-clear      # When clearing
```

#### Claude hooks in settings.json

Update to use simple commands:

```json
"UserPromptSubmit": "set-workspace-status busy"
"Stop": "set-workspace-status complete"
"SubagentStop": "set-workspace-status complete"
"SessionStart": "set-workspace-status ready"
"SessionEnd": "set-workspace-status exited"
```

## Implementation Steps

### Step 1: Create `set-workspace-status`

- [ ] Copy and rename claude-workspace-status
- [ ] Implement state subsystem logic (ai-_, task-_)
- [ ] Add priority resolution
- [ ] Test with all state combinations

### Step 2: Split i3-voyager-rgb

- [ ] Extract i3 monitoring code → i3-workspace-monitor
- [ ] Simplify remaining code → voyager-workspace-status-daemon
- [ ] Remove state management from daemon
- [ ] Test both components work together

### Step 3: Update existing scripts

- [ ] Update `long` command to use set-workspace-status
- [ ] Update Claude hooks in settings.json
- [ ] Remove old claude-workspace-status

### Step 4: Create/update symlinks

- [ ] Link set-workspace-status to ~/.local/bin
- [ ] Link i3-workspace-monitor to ~/.local/bin
- [ ] Link voyager-workspace-status-daemon to ~/.local/bin
- [ ] Remove old symlinks

### Step 5: Testing

- [ ] Test AI states (ready, busy, complete, exited)
- [ ] Test task states (running, succeeded, failed, clear)
- [ ] Test focus changes clear ai-complete
- [ ] Test LED colors match expected states
- [ ] Test Claude hooks work correctly
- [ ] Test long command works correctly

## State Priority (for LED display)

1. urgent (red)
2. task-failed (red)
3. ai-busy (yellow)
4. task-running (yellow)
5. ai-complete (blue)
6. task-succeeded (blue)
7. ai-ready (green)
8. occupied (white)
9. focused (bright version of color)
10. empty (off)

## Benefits of New Architecture

- **Clear separation of concerns**: Each component has one job
- **Single source of truth**: Only set-workspace-status writes to JSON
- **Consistent state management**: All priority logic in one place
- **Easier testing**: Components can be tested independently
- **Extensible**: Easy to add new state subsystems
- **Maintainable**: Clear naming and responsibilities
