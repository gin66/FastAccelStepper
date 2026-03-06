const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ESP32 Stepper Control</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body { font-family: Arial, sans-serif; background: #1a1a1a; color: #fff; padding: 20px; }
        .container { max-width: 1200px; margin: 0 auto; }
        h1 { color: #4CAF50; font-size: 20px; }
        .header-row { display: flex; align-items: center; gap: 30px; margin-bottom: 15px; }
        .header-row h1 { margin: 0; }
        .card { background: #2a2a2a; padding: 15px; margin-bottom: 15px; border-radius: 8px; }
        .card h2 { margin-bottom: 15px; color: #4CAF50; font-size: 18px; }
        .stepper-grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(300px, 1fr)); gap: 15px; }
        .pin-grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(180px, 1fr)); gap: 10px; max-height: 400px; overflow-y: auto; }
        .pin-grid.expanded { max-height: none; }
        .pin-item { background: #333; padding: 10px; border-radius: 5px; font-size: 13px; }
        .pin-item.reserved { opacity: 0.5; }
        .pin-item.assigned { border-left: 3px solid #4CAF50; }
        .pin-header { display: flex; justify-content: space-between; margin-bottom: 5px; }
        .pin-num { font-weight: bold; color: #4CAF50; }
        .pin-value { font-weight: bold; }
        .pin-value.high { color: #4CAF50; }
        .pin-value.low { color: #888; }
        .pin-actions { display: flex; gap: 5px; margin-top: 5px; }
        .pin-actions button { padding: 4px 8px; font-size: 12px; }
        select { padding: 4px; border-radius: 3px; border: 1px solid #555; background: #333; color: #fff; font-size: 12px; }
        .stepper { background: #333; padding: 15px; border-radius: 5px; }
        .stepper h3 { margin-bottom: 10px; font-size: 16px; }
        .status { display: flex; justify-content: space-between; margin-bottom: 10px; font-size: 14px; }
        .status-label { color: #888; }
        .status-value { font-weight: bold; }
        .controls { display: flex; gap: 10px; flex-wrap: wrap; margin-top: 10px; }
        button { padding: 8px 16px; border: none; border-radius: 4px; cursor: pointer; font-size: 14px; }
        .btn-primary { background: #4CAF50; color: white; }
        .btn-danger { background: #f44336; color: white; }
        .btn-secondary { background: #555; color: white; }
        button:hover { opacity: 0.8; }
        input[type="number"] { width: 100px; padding: 5px; border-radius: 3px; border: 1px solid #555; background: #333; color: #fff; }
        .connection-status { display: inline-block; width: 10px; height: 10px; border-radius: 50%; margin-right: 5px; }
        .connected { background: #4CAF50; }
        .disconnected { background: #f44336; }
        .system-info { font-size: 14px; color: #888; }
        .system-info div { margin: 5px 0; }
        .error { color: #f44336; padding: 10px; background: #2a1a1a; border-radius: 4px; margin: 10px 0; }
        .modal { display: none; position: fixed; top: 0; left: 0; width: 100%; height: 100%; background: rgba(0,0,0,0.8); z-index: 1000; }
        .modal.show { display: flex; align-items: center; justify-content: center; }
        .modal-content { background: #2a2a2a; padding: 25px; border-radius: 8px; max-width: 400px; width: 90%; }
        .modal-content h3 { margin-bottom: 20px; color: #4CAF50; }
        .form-group { margin-bottom: 15px; }
        .form-group label { display: block; margin-bottom: 5px; color: #888; font-size: 14px; }
        .form-group input { width: 100%; padding: 8px; border-radius: 3px; border: 1px solid #555; background: #333; color: #fff; }
        .form-actions { display: flex; gap: 10px; margin-top: 20px; }
        .stepper-header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 10px; }
        .stepper-header h3 { margin: 0; }
        .stepper-config { font-size: 12px; color: #888; margin-bottom: 10px; }
        .sequence-item { background: #333; padding: 10px; border-radius: 5px; margin-bottom: 10px; }
        .sequence-header { display: flex; justify-content: space-between; align-items: center; }
        .seq-grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(250px, 1fr)); gap: 15px; }
        .cmd-list { max-height: 200px; overflow-y: auto; margin: 10px 0; }
        .cmd-item { background: #2a2a2a; padding: 8px; margin: 5px 0; border-radius: 3px; font-size: 12px; }
    </style>
</head>
<body>
    <div class="container">
        <div class="header-row">
            <h1>ESP32 Stepper Control</h1>
            <div class="system-info" style="display:flex;gap:20px;align-items:center">
                <span class="connection-status" id="wsStatus"></span>
                <span>IP: <strong id="ip">-</strong></span>
                <span>Heap: <strong id="heap">-</strong></span>
                <span>Up: <strong id="uptime">-</strong></span>
            </div>
        </div>
        
        <div class="card">
            <h2>I2S Expander Module</h2>
            <div id="i2sConfig">
                <div class="status">
                    <span class="status-label">Status:</span>
                    <span class="status-value" id="i2sStatus">Loading...</span>
                </div>
                <div class="status" id="i2sPinsInfo" style="display:none">
                    <span class="status-label">Pins:</span>
                    <span class="status-value" id="i2sPins">-</span>
                </div>
                <div class="controls" style="margin-top:10px">
                    <button class="btn-primary" onclick="showI2SModal()">Configure I2S</button>
                </div>
                <div id="i2sPinsTable" style="display:none; margin-top:15px">
                    <h3 style="font-size:14px; margin-bottom:10px; color:#4CAF50">I2S Output Pins (click to toggle)</h3>
                    <div id="i2sPinGrid" style="display:grid; grid-template-columns:repeat(16, 1fr); gap:5px; font-size:11px"></div>
                </div>
            </div>
        </div>
        
        <div class="card">
            <h2>Stepper Motors</h2>
            <div class="controls" style="margin-bottom:15px">
                <button class="btn-primary" onclick="showStepperModal()">Add Stepper</button>
            </div>
            <div id="steppers" class="stepper-grid">Loading...</div>
        </div>
        
        <div class="card">
            <h2>GPIO Pins <span style="font-weight:normal;color:#888;font-size:12px">(click IN/OUT to toggle mode, click value to toggle 0/1)</span></h2>
            <div id="pins" style="display:grid;grid-template-columns:repeat(8,1fr);gap:5px;font-size:11px">Loading...</div>
        </div>
        
        <div class="card">
            <h2>Sequences</h2>
            <div class="controls" style="margin-bottom:15px">
                <button class="btn-primary" onclick="showSequenceModal()">Add Sequence</button>
            </div>
            <div id="sequences">Loading...</div>
        </div>
    </div>

    <div id="stepperModal" class="modal">
        <div class="modal-content" style="max-width:450px">
            <h3 id="modalTitle">Add Stepper</h3>
            <input type="hidden" id="editStepperId" value="">
            <div class="form-group">
                <label>Name</label>
                <input type="text" id="stepperName" placeholder="My Stepper">
            </div>
            <div class="form-group">
                <label>Driver</label>
                <select id="stepperDriver" onchange="updatePinOptions()">
                    <option value="RMT">RMT</option>
                    <option value="I2S_MUX">I2S MUX</option>
                    <option value="I2S_DIRECT">I2S Direct</option>
                </select>
            </div>
            <div class="form-group">
                <label>Step Pin</label>
                <select id="stepperStepPin"></select>
            </div>
            <div class="form-group">
                <label>Direction Pin</label>
                <select id="stepperDirPin"></select>
            </div>
            <div class="form-group">
                <label>Enable Pin (active low)</label>
                <select id="stepperEnableLowPin"></select>
            </div>
            <div class="form-group">
                <label>Enable Pin (active high)</label>
                <select id="stepperEnableHighPin"></select>
            </div>
            <div class="form-group">
                <label>Speed (us/step)</label>
                <input type="number" id="stepperSpeed" value="50" min="1">
            </div>
            <div class="form-group">
                <label>Acceleration</label>
                <input type="number" id="stepperAccel" value="10000" min="1">
            </div>
            <div class="form-group">
                <label>Auto Enable</label>
                <select id="stepperAutoEnable">
                    <option value="false">No</option>
                    <option value="true">Yes</option>
                </select>
            </div>
            <div class="form-actions">
                <button class="btn-primary" onclick="saveStepper()">Save</button>
                <button class="btn-secondary" onclick="closeStepperModal()">Cancel</button>
                <button class="btn-danger" id="deleteBtn" style="display:none" onclick="deleteStepper()">Delete</button>
            </div>
        </div>
    </div>

    <div id="sequenceModal" class="modal">
        <div class="modal-content" style="max-width:500px">
            <h3 id="seqModalTitle">Add Sequence</h3>
            <input type="hidden" id="editSequenceId" value="">
            <div class="form-group">
                <label>Name</label>
                <input type="text" id="seqName" placeholder="My Sequence">
            </div>
            <div class="form-group">
                <label>Loop</label>
                <select id="seqLoop">
                    <option value="false">No</option>
                    <option value="true">Yes</option>
                </select>
            </div>
            <div class="form-group">
                <label>Commands (one per line: stepper_id,command,value,delay_ms)</label>
                <textarea id="seqCommands" style="width:100%;height:150px;background:#333;color:#fff;border:1px solid #555;border-radius:3px;padding:8px;font-family:monospace;font-size:12px" placeholder="0,move,1000,0&#10;0,wait,0,500&#10;0,move_to,0,0"></textarea>
            </div>
            <div class="form-actions">
                <button class="btn-primary" onclick="saveSequence()">Save</button>
                <button class="btn-secondary" onclick="closeSequenceModal()">Cancel</button>
                <button class="btn-danger" id="deleteSeqBtn" style="display:none" onclick="deleteSequence()">Delete</button>
            </div>
        </div>
    </div>

    <div id="i2sModal" class="modal">
        <div class="modal-content">
            <h3>I2S Expander Configuration</h3>
            <div class="form-group">
                <label>Enable I2S Expander (32 outputs)</label>
                <select id="i2sEnable">
                    <option value="false">Disabled</option>
                    <option value="true">Enabled</option>
                </select>
            </div>
            <div class="form-group">
                <label>Data Pin</label>
                <input type="number" id="i2sDataPin" value="32" min="0" max="39">
            </div>
            <div class="form-group">
                <label>BCLK Pin</label>
                <input type="number" id="i2sBclkPin" value="33" min="0" max="39">
            </div>
            <div class="form-group">
                <label>WS Pin</label>
                <input type="number" id="i2sWsPin" value="14" min="0" max="39">
            </div>
            <div class="form-actions">
                <button class="btn-primary" onclick="saveI2S()">Save</button>
                <button class="btn-secondary" onclick="closeI2SModal()">Cancel</button>
            </div>
        </div>
    </div>

    <script>
        let ws;
        let statusInterval;
        let i2sEnabled = false;
        let gpioPins = [];
        
        function connectWebSocket() {
            ws = new WebSocket(`ws://${location.host}/ws`);
            
            ws.onopen = () => {
                document.getElementById('wsStatus').className = 'connection-status connected';
                console.log('WebSocket connected');
            };
            
            ws.onclose = () => {
                document.getElementById('wsStatus').className = 'connection-status disconnected';
                console.log('WebSocket disconnected');
                setTimeout(connectWebSocket, 3000);
            };
            
            ws.onerror = (error) => {
                console.error('WebSocket error:', error);
            };
            
            ws.onmessage = (event) => {
                const data = JSON.parse(event.data);
                if (data.type === 'status') {
                    updatePositions(data.positions);
                }
            };
        }
        
        function updatePositions(positions) {
            positions.forEach(pos => {
                const posEl = document.getElementById(`pos-${pos.id}`);
                const runningEl = document.getElementById(`running-${pos.id}`);
                const speedEl = document.getElementById(`speed-${pos.id}`);
                
                if (posEl) posEl.textContent = pos.position;
                if (runningEl) runningEl.textContent = pos.running ? 'Running' : 'Stopped';
                if (speedEl) speedEl.textContent = pos.speed || '-';
            });
        }
        
        async function loadSteppers() {
            try {
                const response = await fetch('/api/steppers');
                const data = await response.json();
                displaySteppers(data.steppers || []);
            } catch (error) {
                console.error('Failed to load steppers:', error);
                document.getElementById('steppers').innerHTML = '<div class="error">Failed to load steppers</div>';
            }
        }
        
        function displaySteppers(steppers) {
            const container = document.getElementById('steppers');
            
            if (steppers.length === 0) {
                container.innerHTML = '<div>No steppers configured. Click "Add Stepper" to create one.</div>';
                return;
            }
            
            container.innerHTML = steppers.map(s => {
                const driverInfo = s.driver || 'RMT';
                const stepSource = s.step_source || 'GPIO';
                const dirSource = s.dir_source || 'GPIO';
                const enableLowSource = s.enable_pin_low_source || 'GPIO';
                const enableHighSource = s.enable_pin_high_source || 'GPIO';
                const stepPinStr = (stepSource === 'I2S' ? 'I2S ' : 'GPIO ') + s.step_pin;
                const dirPinStr = s.dir_pin ? ((dirSource === 'I2S' ? 'I2S ' : 'GPIO ') + s.dir_pin) : '-';
                const enableLowPinStr = s.enable_pin_low ? ((enableLowSource === 'I2S' ? 'I2S ' : 'GPIO ') + s.enable_pin_low) : '-';
                const enableHighPinStr = s.enable_pin_high ? ((enableHighSource === 'I2S' ? 'I2S ' : 'GPIO ') + s.enable_pin_high) : '-';
                
                return `
                <div class="stepper">
                    <div class="stepper-header">
                        <h3>${s.name || 'Stepper ' + s.id}</h3>
                        <button class="btn-secondary" onclick="editStepper(${s.id}, '${s.name || ''}', '${driverInfo}', ${s.step_pin}, '${stepSource}', ${s.dir_pin || 0}, '${dirSource}', ${s.enable_pin_low || 0}, '${enableLowSource}', ${s.enable_pin_high || 0}, '${enableHighSource}', ${s.speed_us}, ${s.acceleration}, ${s.auto_enable || false})">Edit</button>
                    </div>
                    <div class="stepper-config">Driver: ${driverInfo} | Step: ${stepPinStr} | Dir: ${dirPinStr}</div>
                    <div class="stepper-config">Enable Low: ${enableLowPinStr} | Enable High: ${enableHighPinStr}</div>
                    <div class="stepper-config">Speed: ${s.speed_us}us | Accel: ${s.acceleration} | Auto Enable: ${s.auto_enable ? 'Yes' : 'No'}</div>
                    <div class="status">
                        <span class="status-label">Position:</span>
                        <span class="status-value" id="pos-${s.id}">-</span>
                    </div>
                    <div class="status">
                        <span class="status-label">Status:</span>
                        <span class="status-value" id="running-${s.id}">-</span>
                    </div>
                    <div class="status">
                        <span class="status-label">Speed:</span>
                        <span class="status-value" id="speed-${s.id}">-</span>
                    </div>
                    <div class="controls">
                        <button class="btn-primary" onclick="sendStepperCommand(${s.id}, 'run_forward')">Fwd</button>
                        <button class="btn-primary" onclick="sendStepperCommand(${s.id}, 'run_backward')">Rev</button>
                        <button class="btn-danger" onclick="sendStepperCommand(${s.id}, 'stop')">Stop</button>
                        <button class="btn-secondary" onclick="sendStepperCommand(${s.id}, 'disable')">Disable</button>
                    </div>
                </div>
                `;
            }).join('');
        }
        
        async function sendStepperCommand(stepperId, command, value1) {
            try {
                const params = new URLSearchParams({
                    stepper_id: stepperId,
                    command: command
                });
                if (value1 !== undefined) {
                    params.append('value1', value1);
                }
                
                const response = await fetch('/api/steppers/command', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/x-www-form-urlencoded'},
                    body: params
                });
                const data = await response.json();
                if (!data.success) {
                    alert('Error: ' + data.error);
                }
            } catch (error) {
                alert('Failed to send command: ' + error.message);
            }
        }
        
        async function updateSystemStatus() {
            try {
                const response = await fetch('/api/status');
                const data = await response.json();
                
                document.getElementById('ip').textContent = data.wifi.ip;
                document.getElementById('heap').textContent = data.heap.free.toLocaleString();
                
                const uptime = Math.floor(data.uptime_ms / 1000);
                const hours = Math.floor(uptime / 3600);
                const minutes = Math.floor((uptime % 3600) / 60);
                const seconds = uptime % 60;
                document.getElementById('uptime').textContent = 
                    `${hours}:${minutes.toString().padStart(2, '0')}:${seconds.toString().padStart(2, '0')}`;
            } catch (error) {
                console.error('Failed to update system status:', error);
            }
        }
        
        connectWebSocket();
        loadSteppers();
        loadPins();
        updateSystemStatus();
        setInterval(updateSystemStatus, 5000);
        
        async function loadPins() {
            try {
                const response = await fetch('/api/pins');
                const data = await response.json();
                gpioPins = (data.pins || []).filter(p => p.source !== 'i2s');
                displayPins(data.pins || []);
            } catch (error) {
                console.error('Failed to load pins:', error);
                document.getElementById('pins').innerHTML = '<div class="error">Failed to load pins</div>';
            }
        }
        
        async function loadI2SStatus() {
            try {
                const response = await fetch('/api/i2s');
                const data = await response.json();
                i2sEnabled = data.enabled || false;
            } catch (error) {
                i2sEnabled = false;
            }
        }
        
        function getPinOptions(selectedValue, pinType = 'any', driver = 'RMT') {
            let options = '<option value="">-</option>';
            
            if (driver === 'I2S_MUX' && pinType === 'step') {
                if (i2sEnabled) {
                    for (let i = 0; i < 32; i++) {
                        const sel = (selectedValue === 'I2S' + i) ? 'selected' : '';
                        options += `<option value="I2S${i}" ${sel}>I2S ${i}</option>`;
                    }
                }
            } else {
                const usablePins = gpioPins.filter(p => !p.is_reserved && !p.is_assigned);
                for (const p of usablePins) {
                    const sel = (selectedValue === 'GPIO' + p.pin) ? 'selected' : '';
                    options += `<option value="GPIO${p.pin}" ${sel}>GPIO ${p.pin}</option>`;
                }
                
                if (i2sEnabled) {
                    options += '<optgroup label="I2S">';
                    for (let i = 0; i < 32; i++) {
                        const sel = (selectedValue === 'I2S' + i) ? 'selected' : '';
                        options += `<option value="I2S${i}" ${sel}>I2S ${i}</option>`;
                    }
                    options += '</optgroup>';
                }
            }
            
            return options;
        }
        
        function updatePinOptions() {
            const driver = document.getElementById('stepperDriver').value;
            const stepPin = document.getElementById('stepperStepPin');
            const dirPin = document.getElementById('stepperDirPin');
            const enableLowPin = document.getElementById('stepperEnableLowPin');
            const enableHighPin = document.getElementById('stepperEnableHighPin');
            const stepVal = stepPin.value;
            const dirVal = dirPin.value;
            const enableLowVal = enableLowPin.value;
            const enableHighVal = enableHighPin.value;
            
            stepPin.innerHTML = getPinOptions(stepVal, 'step', driver);
            dirPin.innerHTML = getPinOptions(dirVal, 'dir', driver);
            enableLowPin.innerHTML = getPinOptions(enableLowVal, 'enable_low', driver);
            enableHighPin.innerHTML = getPinOptions(enableHighVal, 'enable_high', driver);
        }
        
        function displayPins(pins) {
            const container = document.getElementById('pins');
            const usablePins = pins.filter(p => !p.is_reserved && p.source !== 'i2s');
            
            if (usablePins.length === 0) {
                container.innerHTML = '<div>No usable GPIO pins available</div>';
                return;
            }
            
            let html = '';
            for (const p of usablePins) {
                const canOutput = p.capabilities & 2;
                const isOutput = p.type === 2;
                const modeStr = isOutput ? 'OUT' : 'IN';
                const value = p.value === -1 ? '-' : p.value;
                const isAssigned = p.is_assigned;
                const bgColor = isOutput && value === 1 ? '#4CAF50' : '#333';
                const opacity = isAssigned ? '0.5' : '1';
                const modeCursor = (isAssigned || !canOutput) ? 'default' : 'pointer';
                const assignedTitle = isAssigned ? ` (S${p.assigned_to})` : '';
                const inputOnly = !canOutput ? ' (IN only)' : '';
                
                html += `
                    <div style="background:${bgColor};padding:5px;border-radius:3px;text-align:center;opacity:${opacity}"
                         title="GPIO${p.pin}${assignedTitle}${inputOnly}">
                        <div style="font-weight:bold;font-size:10px;color:#888">${p.pin}</div>
                        <div style="display:flex;justify-content:center;gap:8px;font-weight:bold">
                            <span style="cursor:${modeCursor};color:${canOutput ? '#fff' : '#888'}" 
                                  onclick="${isAssigned || !canOutput ? '' : `cyclePinMode(${p.pin})`}">${modeStr}</span>
                            <span style="cursor:${isOutput && !isAssigned ? 'pointer' : 'default'};color:${isOutput && value === 1 ? '#fff' : '#888'}" 
                                  onclick="${isOutput && !isAssigned ? `togglePin(${p.pin})` : ''}">${value}</span>
                        </div>
                    </div>
                `;
            }
            container.innerHTML = html;
        }
        
        async function cyclePinMode(pin) {
            const pinData = gpioPins.find(p => p.pin === pin);
            if (!pinData) return;
            
            const newMode = pinData.type === 2 ? 'input' : 'output';
            
            try {
                const params = new URLSearchParams({ mode: newMode, value: '0' });
                await fetch(`/api/pins/${pin}`, {
                    method: 'POST',
                    headers: {'Content-Type': 'application/x-www-form-urlencoded'},
                    body: params
                });
                loadPins();
            } catch (error) {
                console.error('Failed to set pin mode:', error);
            }
        }
        
        async function togglePin(pin) {
            try {
                await fetch(`/api/pins/${pin}/toggle`, { method: 'POST' });
                loadPins();
            } catch (error) {
                console.error('Failed to toggle pin:', error);
            }
        }
        
        function showStepperModal(editId = null) {
            document.getElementById('stepperModal').classList.add('show');
            document.getElementById('editStepperId').value = (editId !== null && editId !== undefined) ? editId : '';
            updatePinOptions();
            
            if (editId !== null) {
                document.getElementById('modalTitle').textContent = 'Edit Stepper';
                document.getElementById('deleteBtn').style.display = 'inline-block';
            } else {
                document.getElementById('modalTitle').textContent = 'Add Stepper';
                document.getElementById('deleteBtn').style.display = 'none';
                document.getElementById('stepperName').value = '';
                document.getElementById('stepperDriver').value = 'RMT';
                document.getElementById('stepperStepPin').value = '';
                document.getElementById('stepperDirPin').value = '';
                document.getElementById('stepperEnableLowPin').value = '';
                document.getElementById('stepperEnableHighPin').value = '';
                document.getElementById('stepperSpeed').value = '50';
                document.getElementById('stepperAccel').value = '10000';
                document.getElementById('stepperAutoEnable').value = 'false';
            }
        }
        
        function closeStepperModal() {
            document.getElementById('stepperModal').classList.remove('show');
        }
        
        function editStepper(id, name, driver, stepPin, stepSource, dirPin, dirSource, enableLowPin, enableLowSource, enableHighPin, enableHighSource, speed, accel, autoEnable) {
            document.getElementById('stepperName').value = name;
            document.getElementById('stepperDriver').value = driver || 'RMT';
            document.getElementById('stepperSpeed').value = speed;
            document.getElementById('stepperAccel').value = accel;
            document.getElementById('stepperAutoEnable').value = autoEnable ? 'true' : 'false';
            updatePinOptions();
            document.getElementById('stepperStepPin').value = (stepSource === 'I2S' ? 'I2S' : 'GPIO') + stepPin;
            document.getElementById('stepperDirPin').value = dirPin ? ((dirSource === 'I2S' ? 'I2S' : 'GPIO') + dirPin) : '';
            document.getElementById('stepperEnableLowPin').value = enableLowPin ? ((enableLowSource === 'I2S' ? 'I2S' : 'GPIO') + enableLowPin) : '';
            document.getElementById('stepperEnableHighPin').value = enableHighPin ? ((enableHighSource === 'I2S' ? 'I2S' : 'GPIO') + enableHighPin) : '';
            showStepperModal(id);
        }
        
        async function saveStepper() {
            const editId = document.getElementById('editStepperId').value;
            const name = document.getElementById('stepperName').value;
            const driver = document.getElementById('stepperDriver').value;
            const stepPinRaw = document.getElementById('stepperStepPin').value;
            const dirPinRaw = document.getElementById('stepperDirPin').value;
            const enableLowPinRaw = document.getElementById('stepperEnableLowPin').value;
            const enableHighPinRaw = document.getElementById('stepperEnableHighPin').value;
            const speed = document.getElementById('stepperSpeed').value;
            const accel = document.getElementById('stepperAccel').value;
            const autoEnable = document.getElementById('stepperAutoEnable').value;
            
            if (!name || !stepPinRaw) {
                alert('Name and Step Pin are required');
                return;
            }
            
            const stepSource = stepPinRaw.startsWith('I2S') ? 'I2S' : 'GPIO';
            const stepPin = stepPinRaw.replace(/^(GPIO|I2S)/, '');
            const dirSource = dirPinRaw ? (dirPinRaw.startsWith('I2S') ? 'I2S' : 'GPIO') : '';
            const dirPin = dirPinRaw ? dirPinRaw.replace(/^(GPIO|I2S)/, '') : '';
            const enableLowSource = enableLowPinRaw ? (enableLowPinRaw.startsWith('I2S') ? 'I2S' : 'GPIO') : '';
            const enableLowPin = enableLowPinRaw ? enableLowPinRaw.replace(/^(GPIO|I2S)/, '') : '';
            const enableHighSource = enableHighPinRaw ? (enableHighPinRaw.startsWith('I2S') ? 'I2S' : 'GPIO') : '';
            const enableHighPin = enableHighPinRaw ? enableHighPinRaw.replace(/^(GPIO|I2S)/, '') : '';
            
            const params = new URLSearchParams({
                name: name,
                driver: driver,
                step_pin: stepPin,
                step_source: stepSource,
                speed_us: speed,
                acceleration: accel,
                auto_enable: autoEnable
            });
            if (dirPin) {
                params.append('dir_pin', dirPin);
                params.append('dir_source', dirSource);
            }
            if (enableLowPin) {
                params.append('enable_pin_low', enableLowPin);
                params.append('enable_pin_low_source', enableLowSource);
            }
            if (enableHighPin) {
                params.append('enable_pin_high', enableHighPin);
                params.append('enable_pin_high_source', enableHighSource);
            }
            
            try {
                let url = '/api/steppers';
                let method = 'POST';
                
                if (editId) {
                    url = `/api/steppers/${editId}`;
                    method = 'PUT';
                }
                
                const response = await fetch(url, {
                    method: method,
                    headers: {'Content-Type': 'application/x-www-form-urlencoded'},
                    body: params
                });
                
                const data = await response.json();
                if (data.success) {
                    closeStepperModal();
                    loadSteppers();
                    if (data.message && data.message.includes('Restart')) {
                        alert(data.message);
                    }
                } else {
                    alert('Error: ' + data.error);
                }
            } catch (error) {
                alert('Failed to save stepper: ' + error.message);
            }
        }
        
        async function deleteStepper() {
            const editId = document.getElementById('editStepperId').value;
            if (!editId) return;
            
            if (!confirm('Delete this stepper?')) return;
            
            try {
                const response = await fetch(`/api/steppers/${editId}`, { method: 'DELETE' });
                const data = await response.json();
                if (data.success) {
                    closeStepperModal();
                    loadSteppers();
                } else {
                    alert('Error: ' + data.error);
                }
            } catch (error) {
                alert('Failed to delete stepper: ' + error.message);
            }
        }
        
        async function loadSequences() {
            try {
                const response = await fetch('/api/sequences');
                const data = await response.json();
                displaySequences(data.sequences || []);
            } catch (error) {
                console.error('Failed to load sequences:', error);
                document.getElementById('sequences').innerHTML = '<div class="error">Failed to load sequences</div>';
            }
        }
        
        function displaySequences(seqs) {
            const container = document.getElementById('sequences');
            
            if (seqs.length === 0) {
                container.innerHTML = '<div>No sequences configured.</div>';
                return;
            }
            
            container.innerHTML = seqs.map(s => `
                <div class="stepper" style="display:flex;justify-content:space-between;align-items:center;padding:10px;margin-bottom:10px">
                    <div>
                        <strong>${s.name || 'Sequence ' + s.id}</strong>
                        <span style="color:#888;margin-left:10px">${s.num_commands} commands ${s.loop ? '(loop)' : ''}</span>
                    </div>
                    <div class="controls" style="margin:0">
                        <button class="btn-primary" onclick="runSequence(${s.id})">Run</button>
                        <button class="btn-secondary" onclick="editSequence(${s.id})">Edit</button>
                    </div>
                </div>
            `).join('');
        }
        
        function showSequenceModal(editId) {
            document.getElementById('sequenceModal').classList.add('show');
            document.getElementById('editSequenceId').value = editId || '';
            
            if (editId) {
                document.getElementById('seqModalTitle').textContent = 'Edit Sequence';
                document.getElementById('deleteSeqBtn').style.display = 'inline-block';
            } else {
                document.getElementById('seqModalTitle').textContent = 'Add Sequence';
                document.getElementById('deleteSeqBtn').style.display = 'none';
                document.getElementById('seqName').value = '';
                document.getElementById('seqLoop').value = 'false';
                document.getElementById('seqCommands').value = '';
            }
        }
        
        function closeSequenceModal() {
            document.getElementById('sequenceModal').classList.remove('show');
        }
        
        function editSequence(id) {
            showSequenceModal(id);
        }
        
        async function saveSequence() {
            const editId = document.getElementById('editSequenceId').value;
            const name = document.getElementById('seqName').value;
            const loop = document.getElementById('seqLoop').value === 'true';
            
            if (!name) {
                alert('Name is required');
                return;
            }
            
            alert('Sequence creation via API not yet implemented. Use config file.');
            closeSequenceModal();
        }
        
        async function deleteSequence() {
            const editId = document.getElementById('editSequenceId').value;
            if (!editId) return;
            
            if (!confirm('Delete this sequence?')) return;
            
            try {
                const response = await fetch(`/api/sequences/${editId}`, { method: 'DELETE' });
                const data = await response.json();
                if (data.success) {
                    closeSequenceModal();
                    loadSequences();
                } else {
                    alert('Error: ' + data.error);
                }
            } catch (error) {
                alert('Failed to delete sequence: ' + error.message);
            }
        }
        
        async function runSequence(id) {
            try {
                const response = await fetch(`/api/sequences/${id}/run`, { method: 'POST' });
                const data = await response.json();
                if (!data.success) {
                    alert('Error: ' + data.error);
                }
            } catch (error) {
                alert('Failed to run sequence: ' + error.message);
            }
        }
        
        async function loadI2SConfig() {
            try {
                const response = await fetch('/api/i2s');
                const data = await response.json();
                i2sEnabled = data.enabled || false;
                
                const statusEl = document.getElementById('i2sStatus');
                const pinsInfoEl = document.getElementById('i2sPinsInfo');
                const pinsEl = document.getElementById('i2sPins');
                const pinsTableEl = document.getElementById('i2sPinsTable');
                
                if (i2sEnabled) {
                    statusEl.textContent = 'Enabled (32 outputs)';
                    statusEl.style.color = '#4CAF50';
                    pinsInfoEl.style.display = 'flex';
                    pinsEl.textContent = `Data: ${data.data_pin}, BCLK: ${data.bclk_pin}, WS: ${data.ws_pin}`;
                    
                    if (data.initialized) {
                        pinsTableEl.style.display = 'block';
                        loadI2SPinsTable();
                    } else {
                        pinsTableEl.style.display = 'none';
                    }
                } else {
                    statusEl.textContent = 'Disabled';
                    statusEl.style.color = '#888';
                    pinsInfoEl.style.display = 'none';
                    pinsTableEl.style.display = 'none';
                }
            } catch (error) {
                document.getElementById('i2sStatus').textContent = 'Error loading';
            }
        }
        
        async function loadI2SPinsTable() {
            try {
                const response = await fetch('/api/pins');
                const data = await response.json();
                const i2sPins = (data.pins || []).filter(p => p.source === 'i2s');
                displayI2SPinsTable(i2sPins);
            } catch (error) {
                console.error('Failed to load I2S pins:', error);
            }
        }
        
        function displayI2SPinsTable(pins) {
            const container = document.getElementById('i2sPinGrid');
            
            let html = '';
            for (let i = 0; i < 32; i++) {
                const pin = pins.find(p => p.pin === i);
                const value = pin ? pin.value : 0;
                const isAssigned = pin && pin.is_assigned;
                const valueClass = value === 1 ? 'high' : 'low';
                const bgColor = value === 1 ? '#4CAF50' : '#333';
                const cursor = isAssigned ? 'not-allowed' : 'pointer';
                const opacity = isAssigned ? '0.5' : '1';
                
                html += `
                    <div style="background:${bgColor}; padding:5px; border-radius:3px; text-align:center; cursor:${cursor}; opacity:${opacity}"
                         ${!isAssigned ? `onclick="toggleI2SPin(${i})"` : ''}
                         title="I2S ${i}${isAssigned ? ' (assigned to S' + pin.assigned_to + ')' : ''}">
                        <div style="font-weight:bold; font-size:10px; color:#888">${i}</div>
                        <div class="pin-value ${valueClass}" style="font-weight:bold">${value}</div>
                    </div>
                `;
            }
            container.innerHTML = html;
        }
        
        async function toggleI2SPin(pin) {
            try {
                const params = new URLSearchParams({ source: 'i2s' });
                await fetch(`/api/pins/${pin}/toggle`, {
                    method: 'POST',
                    headers: {'Content-Type': 'application/x-www-form-urlencoded'},
                    body: params
                });
                loadI2SPinsTable();
            } catch (error) {
                console.error('Failed to toggle I2S pin:', error);
            }
        }
        
        function showI2SModal() {
            document.getElementById('i2sModal').classList.add('show');
            loadI2SConfig();
            
            fetch('/api/i2s')
                .then(r => r.json())
                .then(data => {
                    document.getElementById('i2sEnable').value = data.enabled ? 'true' : 'false';
                    document.getElementById('i2sDataPin').value = data.data_pin || 32;
                    document.getElementById('i2sBclkPin').value = data.bclk_pin || 33;
                    document.getElementById('i2sWsPin').value = data.ws_pin || 14;
                });
        }
        
        function closeI2SModal() {
            document.getElementById('i2sModal').classList.remove('show');
        }
        
        async function saveI2S() {
            const enabled = document.getElementById('i2sEnable').value === 'true';
            const dataPin = document.getElementById('i2sDataPin').value;
            const bclkPin = document.getElementById('i2sBclkPin').value;
            const wsPin = document.getElementById('i2sWsPin').value;
            
            const params = new URLSearchParams({
                enabled: enabled,
                data_pin: dataPin,
                bclk_pin: bclkPin,
                ws_pin: wsPin
            });
            
            try {
                const response = await fetch('/api/i2s', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/x-www-form-urlencoded'},
                    body: params
                });
                
                const data = await response.json();
                if (data.success) {
                    closeI2SModal();
                    loadI2SConfig();
                    loadPins();
                    alert('I2S configuration saved. Restart device to apply changes.');
                } else {
                    alert('Error: ' + data.error);
                }
            } catch (error) {
                alert('Failed to save I2S config: ' + error.message);
            }
        }
        
        connectWebSocket();
        loadSteppers();
        loadPins();
        loadI2SConfig();
        loadSequences();
        updateSystemStatus();
        setInterval(updateSystemStatus, 5000);
    </script>
</body>
</html>
)rawliteral";
