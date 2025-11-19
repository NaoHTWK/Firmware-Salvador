import streamlit as st
import socket
import time

DEFAULT_HOST = "localhost"
DEFAULT_PORT = 12345

PARAMETERS = {
    "foot_yaw_L": {
        "label": "Foot Yaw Left",
        "min": -0.7,
        "max": 0.7,
        "default": 0.0,
        "step": 0.01,
        "help": "Left foot yaw angle [-0.7, 0.7]"
    },
    "foot_yaw_R": {
        "label": "Foot Yaw Right",
        "min": -0.1,
        "max": 0.3,
        "default": 0.0,
        "step": 0.01,
        "help": "Right foot yaw angle [-0.1, 0.3]"
    },
    "body_pitch_target": {
        "label": "Body Pitch Target",
        "min": -0.1,
        "max": 0.3,
        "default": 0.0,
        "step": 0.01,
        "help": "Body pitch target angle [-0.1, 0.3]"
    },
    "body_roll_target": {
        "label": "Body Roll Target",
        "min": -0.1,
        "max": 0.1,
        "default": 0.0,
        "step": 0.01,
        "help": "Body roll target angle [-0.1, 0.1]"
    },
    "feet_offset_x_target": {
        "label": "Feet Offset X Target",
        "min": -0.25,
        "max": 0.25,
        "default": 0.0,
        "step": 0.01,
        "help": "Feet offset in X direction [-0.25, 0.25]"
    },
    "feet_offset_y_target": {
        "label": "Feet Offset Y Target",
        "min": -0.2,
        "max": 0.2,
        "default": 0.0,
        "step": 0.01,
        "help": "Feet offset in Y direction [-0.2, 0.2]"
    }
}

def send_command(host, port, command):
    """Send a command to the parameter tuner socket."""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(2.0)
        sock.connect((host, port))
        sock.sendall(command.encode('utf-8'))
        response = sock.recv(1024).decode('utf-8')
        sock.close()
        return True, response.strip()
    except socket.timeout:
        return False, "Connection timeout"
    except ConnectionRefusedError:
        return False, "Connection refused - is the robot running?"
    except Exception as e:
        return False, f"Error: {str(e)}"

def set_parameter(host, port, param_name, value):
    """Set a parameter value."""
    command = f"set {param_name} {value}\n"
    return send_command(host, port, command)

st.set_page_config(page_title="Parameter Tuner", page_icon="⚙️", layout="wide")

st.title("⚙️ Parameter Tuner")
st.markdown("Tune walking parameters for the robot in real-time")

col1, col2 = st.columns([1, 1])

with col1:
    st.subheader("Connection Settings")
    host = st.text_input("Host", value=DEFAULT_HOST, help="Robot IP address or hostname")
    port = st.number_input("Port", min_value=1, max_value=65535, value=DEFAULT_PORT, help="Parameter tuner port")
    
    if st.button("Test Connection", type="secondary"):
        success, message = send_command(host, port, "test\n")
        if success:
            st.success(f"Connected! Response: {message}")
        else:
            st.error(f"Connection failed: {message}")

with col2:
    st.subheader("Quick Actions")
    if st.button("Reset All to Default", type="secondary"):
        st.session_state.reset_params = True
    
    if st.button("Send All Parameters", type="primary"):
        st.session_state.send_all = True

st.divider()

st.subheader("Walk Parameters")

if 'reset_params' in st.session_state and st.session_state.reset_params:
    for param_name in PARAMETERS:
        if param_name in st.session_state:
            del st.session_state[param_name]
    st.session_state.reset_params = False
    st.rerun()

if 'send_all' in st.session_state and st.session_state.send_all:
    success_count = 0
    error_count = 0
    errors = []
    
    for param_name, config in PARAMETERS.items():
        value = st.session_state.get(param_name, config["default"])
        success, message = set_parameter(host, port, param_name, value)
        if success:
            success_count += 1
        else:
            error_count += 1
            errors.append(f"{param_name}: {message}")
        time.sleep(0.05)
    
    if error_count == 0:
        st.success(f"Successfully sent all {success_count} parameters!")
    else:
        st.warning(f"Sent {success_count} parameters, {error_count} failed")
        for error in errors:
            st.error(error)
    
    st.session_state.send_all = False

cols = st.columns(3)

for idx, (param_name, config) in enumerate(PARAMETERS.items()):
    with cols[idx % 3]:
        value = st.slider(
            config["label"],
            min_value=config["min"],
            max_value=config["max"],
            value=st.session_state.get(param_name, config["default"]),
            step=config["step"],
            key=param_name,
            help=config["help"]
        )
        
        col_a, col_b = st.columns([3, 1])
        with col_a:
            st.text(f"Value: {value:.3f}")
        with col_b:
            if st.button("Set", key=f"set_{param_name}"):
                success, message = set_parameter(host, port, param_name, value)
                if success:
                    st.success("✓")
                    time.sleep(0.3)
                    st.rerun()
                else:
                    st.error(f"✗ {message}")

st.divider()

with st.expander("📖 Usage Instructions"):
    st.markdown("""
    ### How to use:
    1. **Set Connection**: Enter the robot's IP address or hostname and port (default: 12345)
    2. **Test Connection**: Click "Test Connection" to verify connectivity
    3. **Adjust Parameters**: Use the sliders to set parameter values
    4. **Set Individual**: Click "Set" next to each parameter to send it immediately
    5. **Set All**: Click "Send All Parameters" to send all current slider values at once
    6. **Reset**: Click "Reset All to Default" to reset all sliders to default values
    
    ### Parameters:
    - **Foot Yaw**: Controls the yaw angle of each foot
    - **Body Pitch/Roll**: Controls the body orientation targets
    - **Feet Offset**: Controls the offset position of the feet
    
    ### Notes:
    - Changes are sent immediately when you click "Set" or "Send All Parameters"
    - The robot must be running with ParameterTuner active for changes to take effect
    - Parameter ranges are enforced by the sliders
    """)

