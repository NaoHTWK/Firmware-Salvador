# Parameter Tuner Streamlit Frontend

A web-based interface for tuning robot walking parameters in real-time.

## Installation

### Create a Virtual Environment

1. **Create the virtual environment:**
   ```bash
   python3 -m venv venv
   ```
   or
   ```bash
   python -m venv venv
   ```

2. **Activate the virtual environment:**
   
   On Linux/Mac:
   ```bash
   source venv/bin/activate
   ```
   
   On Windows:
   ```bash
   venv\Scripts\activate
   ```

3. **Install the required dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

4. **Deactivate when done:**
   ```bash
   deactivate
   ```

## Usage

Start the Streamlit app:

```bash
streamlit run streamlit_app.py
```

The app will open in your browser. Configure the connection settings:

1. **Host**: Enter the robot's IP address or hostname (default: localhost)
2. **Port**: Enter the parameter tuner port (default: 12345)

## Features

- Real-time parameter adjustment with sliders
- Individual parameter setting
- Batch parameter setting (send all at once)
- Connection testing
- Parameter reset to defaults
- Visual feedback for successful/failed operations

## Parameters

The following walking parameters can be tuned:

- **foot_yaw_L**: Left foot yaw angle [-0.7, 0.7]
- **foot_yaw_R**: Right foot yaw angle [-0.1, 0.3]
- **body_pitch_target**: Body pitch target [-0.1, 0.3]
- **body_roll_target**: Body roll target [-0.1, 0.1]
- **feet_offset_x_target**: Feet offset X [-0.25, 0.25]
- **feet_offset_y_target**: Feet offset Y [-0.2, 0.2]

