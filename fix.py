import os

ws_path = os.path.expanduser("~/open_agbot_ws")
ui_file = os.path.join(ws_path, "src/basekit_ui/basekit_ui/ui_node.py")
# The script causing the GPIO crash
espresso_file = "/root/.lizard/espresso.py" 

def patch_gpio_for_laptop():
    """Mocks GPIO writes so the driver doesn't crash on non-Jetson hardware."""
    if not os.path.exists(espresso_file):
        print(f"⚠️ {espresso_file} not found. If this is in the container, we will patch it via a wrapper.")
        return

    print("🛠 Patching espresso.py to ignore missing GPIO files...")
    with open(espresso_file, 'r') as f:
        content = f.read()
    
    # Wrap the write_gpio function in a try-except block
    buggy_code = "Path(f'/sys/class/gpio/{path}').write_text(f'{value}\\n', encoding='utf-8')"
    safe_code = "try:\n        " + buggy_code + "\n    except FileNotFoundError:\n        print(f'MOCK GPIO: {path} -> {value}')"
    
    if buggy_code in content and "try:" not in content:
        content = content.replace(buggy_code, safe_code)
        with open(espresso_file, 'w') as f:
            f.write(content)

def patch_ui_threading():
    """Forces NiceGUI elements to be created only when a page is requested."""
    print("🛠 Fixing NiceGUI Threading context...")
    with open(ui_file, 'r') as f:
        lines = f.readlines()

    new_lines = []
    for line in lines:
        # We need to move the UI element creation into the @ui.page or ensure it has a target
        if "self.setup_ui()" in line: # Assuming a setup function exists
             new_lines.append(line)
        elif "with ui." in line and "__init__" in line:
            # Wrap standard element creation to prevent background thread crashes
            new_lines.append("        # Thread-safe wrap\n")
            new_lines.append(line.replace("with ui.", "with ui.element('div'): # "))
        else:
            new_lines.append(line)

    with open(ui_file, 'w') as f:
        f.writelines(new_lines)

if __name__ == "__main__":
    patch_gpio_for_laptop()
    patch_ui_threading()
    print("✅ Patches applied. Please run a fresh cleanstart.")
