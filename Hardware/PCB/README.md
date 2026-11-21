# PCB Quick Start

KiCad projects for the WAAR drone electronics. Everything here is **KiCad 9.0**. Opening a file in KiCad 9 upgrades it, and KiCad 8 can't open it afterwards. Don't open these files in an older version.

## Folder layout

```
Hardware/PCB/
├── LIBRARIES/                 shared team libraries (use these for new parts)
│   ├── symbols/               NN_Category.kicad_sym
│   ├── footprints/            NN_Category.pretty/
│   └── 3dmodels/              NN_Category/*.step
└── FLIGHT_CONTROLLER/         one folder per board
    ├── FLIGHT_CONTROLLER.kicad_pro / .kicad_sch / .kicad_pcb
    ├── sym-lib-table, fp-lib-table     project library tables (committed)
    ├── libraries/             parts that belong only to this board
    └── production/            fab outputs (gerbers zip, BOM, positions)
```

## 1. One-time setup

1. **Install KiCad 9.0** from kicad.org. The schematic also uses KiCad's built-in libraries (`Device`, `power`, `Connector_Generic`, and others). They come with the install.
2. **Install Git LFS** and clone the repo:
   ```
   git lfs install
   git clone https://github.com/Washington-Aerial-Robotics/waar-iarc-m10.git
   ```
   Git LFS stores `.kicad_sch` and `.kicad_pcb`, and they're lockable. If you open one and it shows three lines starting with `version https://git-lfs...`, LFS isn't installed. Run `git lfs pull`.
3. **Install the SparkFun library.** Many capacitor, resistor, and LED 3D models use it. Open the Plugin and Content Manager in KiCad and install "SparkFun KiCad Libraries". You can also clone https://github.com/sparkfun/SparkFun-KiCad-Libraries.
4. **Set the path variables.** In KiCad, go to *Preferences → Configure Paths* and add:

   | Name | Value |
   |---|---|
   | `CUSTOM_MODEL` | `<repo>/Hardware/PCB/LIBRARIES/3dmodels` |
   | `SPARKFUN_KICAD_LIBRARY` | the folder where step 3 installed SparkFun |

   If these aren't set, the board still opens, but some parts have no 3D model in the 3D viewer or in the STEP export for CAD.
5. **Optional, for ordering boards:** install the **Fabrication Toolkit** plugin from the Plugin and Content Manager. `fabrication-toolkit-options.json` holds the settings used to build `production/`.

You **don't** need to add anything to the global symbol or footprint library tables. Each project's `sym-lib-table` and `fp-lib-table` uses `${KIPRJMOD}` to find the shared libraries by relative path, so they work in any clone.

## 2. Working on a board

1. Start from an up-to-date `main` and create a branch named `pcb/<what-you're-doing>`:
   ```
   git switch main && git pull
   git switch -c pcb/my-change
   ```
2. **Lock the files before you edit them.** KiCad files can't be merged, so if two people edit the same file, one person's work gets lost.
   ```
   git lfs locks                                    # see who holds what
   git lfs lock Hardware/PCB/FLIGHT_CONTROLLER/FLIGHT_CONTROLLER.kicad_pcb
   ```
   Lock `.kicad_sch` too if you're changing the schematic.
3. Make your changes. Run ERC (schematic) and DRC (board) and fix the errors.
4. Commit, push, and open a PR into `main`. Unlock the files after the merge:
   ```
   git lfs unlock Hardware/PCB/FLIGHT_CONTROLLER/FLIGHT_CONTROLLER.kicad_pcb
   ```

Each person's machine has its own `*.kicad_prl` files, `*-backups/` folders, and `fp-info-cache`. Don't commit them.

## 3. Managing libraries

**Rule: when a new part could go on more than one board, add it to `LIBRARIES/`, not to KiCad's global libraries or your personal folders.** Other people can't see parts that only exist on your machine.

Adding a part:

1. Choose the category library that fits, such as `02_Sensors` or `03_Connectors`. Use the matching symbol (`symbols/NN_X.kicad_sym`), footprint (`footprints/NN_X.pretty`), and model (`3dmodels/NN_X/`).
2. Put the STEP model in `3dmodels/NN_X/`. In the footprint's 3D settings, reference it as `${CUSTOM_MODEL}/NN_X/part.step`. **Never use an absolute path** like `C:\Users\...`.
3. In the symbol properties, set the symbol's **Footprint** field to `NN_X:FootprintName`, and fill in the datasheet and MPN.
4. If you need a new category, create the next number (`13_...`). Then add it to **both** `sym-lib-table` and `fp-lib-table` in every project that uses it, with the same pattern as the other entries:
   ```
   (lib (name "13_Whatever")(type "KiCad")(uri "${KIPRJMOD}/../LIBRARIES/symbols/13_Whatever.kicad_sym")(options "")(descr ""))
   ```
5. Commit the library change in the same PR as the board change that uses it.

**Board-only parts** go in `<BOARD>/libraries/`. In the flight controller project, three library names point there: `FLIGHT_CONTROLLER`, `Sensors`, and `ESP32_Enhanced` for footprints, and `FLIGHT_CONTROLLER` and `dd` for symbols. The ones marked "Recovered from design" are old names that the current board still uses. Leave them alone, and use `FLIGHT_CONTROLLER` for new board-only parts.

**Starting a new board:** create `Hardware/PCB/<NEW_BOARD>/` and copy `sym-lib-table` and `fp-lib-table` from `FLIGHT_CONTROLLER/`. Delete the "Recovered from design" lines. Your board now has the same team libraries.

## 4. Ordering

Run the Fabrication Toolkit (in the PCB editor's toolbar). It writes the gerber zip, `bom.csv`, and `positions.csv` into `production/`. Name the zip `<BOARD>_<MM_DD_YY>.zip`, like the existing `ESP32_DRONE_WAAR_5_16_26.zip`, and commit it with the design. That way the repo always shows what was actually ordered.
