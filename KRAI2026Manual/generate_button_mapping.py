from docx import Document
from docx.shared import Pt, Inches, RGBColor
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH

doc = Document()

# Title
title = doc.add_heading('KRAI 2026 — Button Mapping', level=0)
title.alignment = WD_ALIGN_PARAGRAPH.CENTER

doc.add_paragraph('Master Board (ESP32-S3) — Controller PS4 DualShock 4')

# ── Helper ──
def add_table(doc, headers, rows):
    table = doc.add_table(rows=1 + len(rows), cols=len(headers))
    table.style = 'Light Grid Accent 1'
    table.alignment = WD_TABLE_ALIGNMENT.LEFT
    for i, h in enumerate(headers):
        cell = table.rows[0].cells[i]
        cell.text = h
        for p in cell.paragraphs:
            for r in p.runs:
                r.bold = True
                r.font.size = Pt(9)
    for ri, row in enumerate(rows):
        for ci, val in enumerate(row):
            cell = table.rows[ri + 1].cells[ci]
            cell.text = str(val)
            for p in cell.paragraphs:
                for r in p.runs:
                    r.font.size = Pt(9)
    return table

# ══════════════════════════════════════════════════════════════════════
# 1. JOYSTICK & DPAD
# ══════════════════════════════════════════════════════════════════════
doc.add_heading('1. Joystick & DPAD', level=1)
add_table(doc,
    ['Input', 'Tanpa modifier', '+ R2 hold', '+ Circle hold'],
    [
        ['Analog kiri (LX/LY)', 'Motion vx/vy (ANALOG mode)', 'Armbox motor Y jog (DPAD mode)', 'Armbox FB toggle (DPAD mode)'],
        ['Analog kanan (RX/RY)', 'Yaw ±1°', 'Motor X/Y jog (gripper)', '—'],
        ['DPAD', 'Motion vx/vy (DPAD mode)', 'Armbox motor Y jog (ANALOG mode)', 'Armbox FB toggle (ANALOG mode)'],
    ])

# ══════════════════════════════════════════════════════════════════════
# 2. TOMBOL
# ══════════════════════════════════════════════════════════════════════
doc.add_heading('2. Tombol', level=1)
add_table(doc,
    ['Tombol', 'Edge/Hold', 'Aksi'],
    [
        ['Cross', '—', 'Kosong'],
        ['Circle', 'Hold + arah (edge)', 'Arm box manual:\nKiri→FB toggle R\nKanan→FB toggle L\nAtas→done L\nBawah→done R'],
        ['Square', 'Hold', 'Freeze motion (vx=0, vy=0)'],
        ['Triangle', 'Hold', 'Servo B manual (hanya saat READY_TO_STAB)'],
        ['L1', 'Hold', 'Slow — interval kirim 100ms, yaw step 150ms, gripper step 5'],
        ['R1', 'Hold', 'Fast — interval kirim 5ms, yaw step 25ms, gripper step 40'],
        ['L2 + analog kanan', 'Hold', 'Yaw snap kardinal (0°/90°/180°/-90°)'],
        ['L2 + R2', 'Analog ≥250', 'Flash lamp fire'],
        ['R2', 'Hold', 'Gripper mode — motor X/Y (analog kanan) + armbox Y jog (analog kiri/DPAD)'],
        ['SHARE', 'Edge', 'Toggle input mode (DPAD ↔ ANALOG)'],
        ['SHARE + TOUCHPAD', 'Hold 5 detik', 'Masuk mode record odom'],
        ['SHARE + TOUCHPAD', 'Edge (mode record ON)', 'Record odom slot'],
        ['OPTIONS', 'Edge (zone 1)', 'SetupZone1 — servo homing + motor Y level 0 + motor X enc 200'],
        ['OPTIONS', 'Edge (zone 2)', 'Toggle modeKinematics (goto ↔ kn)'],
        ['L3 + R3', 'Edge', 'Keluar mode record odom'],
        ['L1+R1+L2+R2', 'Edge', 'Toggle invert input (atas↔bawah, kiri↔kanan)'],
        ['PS', '—', 'Kosong'],
    ])

# ══════════════════════════════════════════════════════════════════════
# 3. ODOM RECORD COMBOS
# ══════════════════════════════════════════════════════════════════════
doc.add_heading('3. Odom Record Combos (mode record ON)', level=1)
add_table(doc,
    ['Combo', 'Target'],
    [
        ['R1 + TOUCHPAD', 'Zone1 slot 0'],
        ['L1 + TOUCHPAD', 'Zone1 slot 1'],
        ['R2 + TOUCHPAD', 'Zone1 slot 2'],
        ['L2 + TOUCHPAD', 'Zone1 slot 3'],
        ['R1 + Triangle', 'Approach slot 0'],
        ['L1 + Square', 'Approach slot 1'],
        ['R2 + Triangle', 'Approach slot 2'],
        ['L2 + Square', 'Approach slot 3'],
        ['Cross + TOUCHPAD', 'Forest WP 2'],
        ['Square + TOUCHPAD', 'Forest WP 6'],
        ['Circle + TOUCHPAD', 'Forest WP 7'],
        ['Triangle + TOUCHPAD', 'Forest WP 11'],
    ])

# ══════════════════════════════════════════════════════════════════════
# 4. SPEED & INTERVAL TABLE
# ══════════════════════════════════════════════════════════════════════
doc.add_heading('4. Speed & Interval', level=1)
add_table(doc,
    ['Mode', 'Yaw step', 'Send interval', 'Gripper step'],
    [
        ['Normal', '50ms', '20ms', '15 enc'],
        ['L1 (Slow)', '150ms', '100ms', '5 enc'],
        ['R1 (Fast)', '25ms', '5ms', '40 enc'],
    ])

# ══════════════════════════════════════════════════════════════════════
# 5. MODE TABLE
# ══════════════════════════════════════════════════════════════════════
doc.add_heading('5. Mode Behavior', level=1)
add_table(doc,
    ['Mode', 'R2 OFF', 'R2 ON'],
    [
        ['DPAD', 'DPAD → motion vx/vy\nL-stick → —\nR-stick → yaw', 'DPAD → armbox Y jog\nL-stick → armbox Y jog\nR-stick → motor X/Y'],
        ['ANALOG', 'L-stick → motion vx/vy\nDPAD → —\nR-stick → yaw', 'L-stick → motion vx/vy\nDPAD → armbox Y jog\nR-stick → motor X/Y'],
    ])

# ══════════════════════════════════════════════════════════════════════
# 6. AUTO (TANPA TOMBOL)
# ══════════════════════════════════════════════════════════════════════
doc.add_heading('6. Auto (tanpa tombol)', level=1)
add_table(doc,
    ['Trigger', 'Aksi'],
    [
        ['Proximity R (slave2)', 'Arm box R: pne ON → motor Y level 4 + motor X -255 → grab'],
        ['Proximity L (slave2)', 'Arm box L: pne ON → motor Y level 4 + motor K -255 → grab'],
        ['Motor Y sampai target', 'Hold PWM 50 (anti-gravitasi) + active=false'],
        ['Motor K kena limit', 'Auto stop via motor k 0'],
        ['BOOT button', 'Toggle alliance BLUE ↔ RED (NVS)'],
    ])

# Save
output_path = r'C:\Users\NITRO 5\Documents\GitHub\robot_manual\KRAI2026Manual\Button_Mapping_KRAI2026.docx'
doc.save(output_path)
print(f'Saved: {output_path}')
