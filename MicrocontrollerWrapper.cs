using System;
using System.Collections.Generic;
using System.IO;
using System.Runtime.InteropServices;
using System.Threading;
using System.Threading.Tasks;
using Python.Runtime;

namespace MicrocontrollerWrapper
{
    /// <summary>
    /// Complete C# wrapper for Python Microcontroller class with optimized async pattern
    /// Only uses async for long-running operations where it provides real value
    /// Supports Windows, Linux, macOS with auto-detection
    /// </summary>
    public class MicrocontrollerWrapper : IDisposable
    {
        private dynamic? _microcontroller;
        private dynamic? _microcontrollerModule;
        private dynamic? _serialDevice;
        private dynamic? _defModule;
        private readonly object _commandLock = new object();
        private bool _disposed = false;
        private static bool _pythonInitialized = false;
        private static readonly object _initLock = new object();

        // Store Python types for conversions
        private dynamic? _homingDirectionEnum;
        private dynamic? _axisClass;
        
        // Stage configuration
        private StageAxisConfig _xAxisConfig;
        private StageAxisConfig _yAxisConfig;
        private StageAxisConfig _zAxisConfig;
        private StageAxisConfig _thetaAxisConfig;
        private const double BACKLASH_COMPENSATION_DISTANCE_MM = 0.005;
        
        // FilterWheel properties
        private int _filterWheelPositionIndex = 1; // Default to min index
        private const int FILTERWHEEL_MIN_INDEX = 1;
        private const int FILTERWHEEL_MAX_INDEX = 8;
        
        // Events
        public event EventHandler<PositionUpdateEventArgs>? PositionUpdated;
        
        /// <summary>
        /// Initialize the microcontroller wrapper with auto-detection support and stage configuration
        /// </summary>
        public MicrocontrollerWrapper(
            string pythonScriptPath, 
            string? portName = null, 
            int baudRate = 2000000, 
            bool simulated = false, 
            bool resetAndInitialize = true,
            StageConfig? stageConfig = null)
        {
            if (!File.Exists(pythonScriptPath))
            {
                throw new FileNotFoundException($"Python script not found: {pythonScriptPath}");
            }

            InitializePython();
            InitializeMicrocontroller(pythonScriptPath, portName, baudRate, simulated, resetAndInitialize);
            
            // Configure stage axes if we have configuration
            if (_defModule != null)
            {
                try
                {
                    using (Py.GIL())
                    {
                        _xAxisConfig = StageAxisConfig.FromPythonDef(_defModule, "X");
                        _yAxisConfig = StageAxisConfig.FromPythonDef(_defModule, "Y");
                        _zAxisConfig = StageAxisConfig.FromPythonDef(_defModule, "Z");
                        _thetaAxisConfig = StageAxisConfig.FromPythonDef(_defModule, "THETA");
                        Console.WriteLine("Successfully loaded stage configuration from Python _def module");
                    }
                }
                catch (Exception ex)
                {
                    throw new InvalidOperationException($"Failed to load stage configuration from Python _def module: {ex}");
                }
            }
            else
            {
                throw new FileNotFoundException("Can not load Python _def module");
            }
        }

        private void ConfigureStageAxes()
        {
            // Configure X axis
            StageConfigureAxis(Axis.X, _xAxisConfig);
            
            // Configure Y axis
            StageConfigureAxis(Axis.Y, _yAxisConfig);
            
            // Configure Z axis
            StageConfigureAxis(Axis.Z, _zAxisConfig);
        }

        private void InitializePython()
        {
            lock (_initLock)
            {
                if (!_pythonInitialized)
                {
                    // Platform-specific Python library detection
                    SetPythonDllPath();
                    
                    PythonEngine.Initialize();
                    PythonEngine.BeginAllowThreads();
                    _pythonInitialized = true;
                }
            }
        }

        private static void SetPythonDllPath()
        {
            // Check if already set
            if (!string.IsNullOrEmpty(Runtime.PythonDLL))
                return;

            string? pythonDll = null;

            if (RuntimeInformation.IsOSPlatform(OSPlatform.Windows))
            {
                pythonDll = FindWindowsPython();
            }
            else if (RuntimeInformation.IsOSPlatform(OSPlatform.Linux))
            {
                pythonDll = FindLinuxPython();
            }
            else if (RuntimeInformation.IsOSPlatform(OSPlatform.OSX))
            {
                pythonDll = FindMacOSPython();
            }

            if (!string.IsNullOrEmpty(pythonDll))
            {
                Console.WriteLine($"Found Python library: {pythonDll}");
                Runtime.PythonDLL = pythonDll;
            }
            else
            {
                throw new InvalidOperationException(
                    "Could not find Python library. Please set the PYTHONNET_PYDLL environment variable " +
                    "or Runtime.PythonDLL to the path of your Python library (e.g., python3.9.so on Linux).");
            }
        }

        private static string? FindLinuxPython()
        {
            var pythonVersions = new[] { "3.11", "3.10", "3.9", "3.8", "3.7" };
            var searchPaths = new[]
            {
                "/usr/lib/x86_64-linux-gnu",
                "/usr/lib64",
                "/usr/lib",
                "/usr/local/lib",
                "/lib/x86_64-linux-gnu",
                "/lib64",
                "/lib"
            };

            foreach (var version in pythonVersions)
            {
                foreach (var path in searchPaths)
                {
                    var variants = new[]
                    {
                        $"libpython{version}.so",
                        $"libpython{version}m.so",
                        $"libpython{version}.so.1",
                        $"libpython{version}.so.1.0"
                    };

                    foreach (var variant in variants)
                    {
                        var libPath = Path.Combine(path, variant);
                        if (File.Exists(libPath))
                            return libPath;
                    }
                }
            }

            return null;
        }

        private static string? FindWindowsPython()
        {
            var pythonVersions = new[] { "311", "310", "39", "38", "37" };
            var searchPaths = new[]
            {
                Environment.GetEnvironmentVariable("PYTHONHOME"),
                @"C:\Python",
                @"C:\Program Files\Python",
                @"C:\Program Files (x86)\Python",
                Environment.GetEnvironmentVariable("LOCALAPPDATA") + @"\Programs\Python"
            };

            foreach (var version in pythonVersions)
            {
                foreach (var basePath in searchPaths)
                {
                    if (string.IsNullOrEmpty(basePath))
                        continue;

                    var pythonPath = Path.Combine(basePath, $"Python{version}", $"python{version}.dll");
                    if (File.Exists(pythonPath))
                        return pythonPath;

                    pythonPath = Path.Combine(basePath, $"python{version}.dll");
                    if (File.Exists(pythonPath))
                        return pythonPath;
                }
            }

            return null;
        }

        private static string? FindMacOSPython()
        {
            var pythonVersions = new[] { "3.11", "3.10", "3.9", "3.8", "3.7" };
            var searchPaths = new[]
            {
                "/usr/local/Frameworks/Python.framework/Versions",
                "/System/Library/Frameworks/Python.framework/Versions",
                "/opt/homebrew/opt/python@",
                "/usr/local/opt/python@"
            };

            foreach (var version in pythonVersions)
            {
                foreach (var path in searchPaths)
                {
                    string libPath;
                    if (path.Contains("@"))
                    {
                        libPath = $"{path}{version}/lib/libpython{version}.dylib";
                    }
                    else
                    {
                        libPath = Path.Combine(path, version, "lib", $"libpython{version}.dylib");
                    }
                    
                    if (File.Exists(libPath))
                        return libPath;
                }
            }

            return null;
        }

        private void InitializeMicrocontroller(string pythonScriptPath, string? portName, int baudRate, bool simulated, bool resetAndInitialize)
        {
            using (Py.GIL())
            {
                try
                {
                    // Get the directory containing the Python script
                    string scriptDir = Path.GetDirectoryName(Path.GetFullPath(pythonScriptPath)) ?? "";
                    
                    // Get the parent directory (one level up)
                    string parentDir = Path.GetDirectoryName(scriptDir) ?? scriptDir;
                    
                    // Import necessary Python modules
                    dynamic sys = Py.Import("sys");
                    dynamic os = Py.Import("os");
                    
                    // Change Python's working directory to parent (one level up)
                    os.chdir(scriptDir);
                    Console.WriteLine($"Changed Python working directory to: {scriptDir}");
                    
                    // Add both parent and script directories to Python path for imports
                    sys.path.insert(0, parentDir);  // Parent directory first
                    sys.path.insert(0, scriptDir);  // Script directory second

                    // Import the microcontroller module
                    string moduleName = Path.GetFileNameWithoutExtension(pythonScriptPath);
                    
                    // If the script is in a subdirectory, we might need to adjust the import
                    // Try direct import first
                    try
                    {
                        _microcontrollerModule = Py.Import(moduleName);
                    }
                    catch
                    {
                        // If direct import fails, try with subdirectory
                        string subDirName = Path.GetFileName(scriptDir);
                        string fullModuleName = $"{subDirName}.{moduleName}";
                        Console.WriteLine($"Trying import as: {fullModuleName}");
                        _microcontrollerModule = Py.Import(fullModuleName);
                    }

                    // Store references to Python classes/enums
                    _homingDirectionEnum = _microcontrollerModule.HomingDirection;
                    _axisClass = _microcontrollerModule.AXIS;
                    
                    // Import control._def module for constants
                    try
                    {
                        _defModule = Py.Import("control._def");
                    }
                    catch
                    {
                        Console.WriteLine("Warning: Could not import control._def module");
                    }

                    // Create serial device
                    if (simulated)
                    {
                        Console.WriteLine("Creating simulated serial device...");
                        _serialDevice = _microcontrollerModule.get_microcontroller_serial_device(
                            simulated: true
                        );
                    }
                    else
                    {
                        // Let Python auto-detect the port if not specified
                        if (string.IsNullOrEmpty(portName))
                        {
                            Console.WriteLine($"Auto-detecting hardware port at {baudRate} baud...");
                            _serialDevice = _microcontrollerModule.get_microcontroller_serial_device(
                                baudrate: baudRate,
                                simulated: false
                            );
                        }
                        else
                        {
                            Console.WriteLine($"Connecting to hardware on port {portName} at {baudRate} baud...");
                            _serialDevice = _microcontrollerModule.get_microcontroller_serial_device(
                                sn: portName,
                                baudrate: baudRate,
                                simulated: false
                            );
                        }
                    }

                    // Create Microcontroller instance
                    _microcontroller = _microcontrollerModule.Microcontroller(
                        _serialDevice,
                        reset_and_initialize: resetAndInitialize
                    );

                    Console.WriteLine("Microcontroller initialized successfully!");
                }
                catch (PythonException ex)
                {
                    throw new InvalidOperationException($"Failed to initialize microcontroller: {ex.Message}", ex);
                }
            }
        }

        #region Stage Methods

        private void StageConfigureAxis(byte axisNumber, StageAxisConfig axisConfig)
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    if (axisConfig.UseEncoder)
                    {
                        _microcontroller?.configure_stage_pid(
                            axisNumber,
                            (int)(axisConfig.ScrewPitch / axisConfig.EncoderStepSize),
                            axisConfig.FlipDirection);
                        
                        if (axisConfig.PidEnabled)
                        {
                            _microcontroller?.set_pid_arguments(
                                axisNumber, axisConfig.PidP, axisConfig.PidI, axisConfig.PidD);
                            _microcontroller?.turn_on_stage_pid(axisNumber);
                        }
                    }
                }
            }
        }

        private double CalcMoveTimeout(double distance, double maxSpeed)
        {
            // We arbitrarily guess that if a move takes 3x the naive "infinite acceleration" time, then it
            // probably timed out. But always use a minimum timeout of at least 3 seconds.
            return Math.Max(3.0, 3.0 * Math.Abs(distance) / Math.Min(0.1, Math.Abs(maxSpeed)));
        }

        public int StageXMmToUsteps(double mm) => _xAxisConfig.ConvertRealUnitsToUstep(mm);
        public int StageYMmToUsteps(double mm) => _yAxisConfig.ConvertRealUnitsToUstep(mm);
        public int StageZMmToUsteps(double mm) => _zAxisConfig.ConvertRealUnitsToUstep(mm);

        public void StageMoveX(double relMm, bool blocking = true)
        {
            MoveXUsteps(_xAxisConfig.ConvertRealUnitsToUstep(relMm));
            if (blocking)
            {
                WaitTillOperationIsCompleted(CalcMoveTimeout(relMm, _xAxisConfig.MaxSpeed));
            }
        }

        public void StageMoveY(double relMm, bool blocking = true)
        {
            MoveYUsteps(_yAxisConfig.ConvertRealUnitsToUstep(relMm));
            if (blocking)
            {
                WaitTillOperationIsCompleted(CalcMoveTimeout(relMm, _yAxisConfig.MaxSpeed));
            }
        }

        public void StageMoveZ(double relMm, bool blocking = true)
        {
            // Backlash compensation for Z axis
            bool needClearBacklash = relMm < 0;

            if (blocking && needClearBacklash)
            {
                double backlashOffset = -BACKLASH_COMPENSATION_DISTANCE_MM;
                double relMoveWithBacklashOffsetMm = relMm + backlashOffset;
                int relMoveWithBacklashOffsetUsteps = _zAxisConfig.ConvertRealUnitsToUstep(relMoveWithBacklashOffsetMm);
                
                MoveZUsteps(relMoveWithBacklashOffsetUsteps);
                if (blocking)
                {
                    WaitTillOperationIsCompleted(
                        CalcMoveTimeout(relMoveWithBacklashOffsetMm, _zAxisConfig.MaxSpeed));
                }
                
                // Second move: the backlash compensation amount
                double backlashCompensationMm = -backlashOffset;
                MoveZUsteps(_zAxisConfig.ConvertRealUnitsToUstep(backlashCompensationMm));
                if (blocking)
                {
                    WaitTillOperationIsCompleted(CalcMoveTimeout(backlashCompensationMm, _zAxisConfig.MaxSpeed));
                }
            }
            else
            {
                // No backlash compensation needed, just move normally
                MoveZUsteps(_zAxisConfig.ConvertRealUnitsToUstep(relMm));
                if (blocking)
                {
                    WaitTillOperationIsCompleted(CalcMoveTimeout(relMm, _zAxisConfig.MaxSpeed));
                }
            }
        }

        public void StageMoveXTo(double absMm, bool blocking = true)
        {
            var currentPos = StageGetPos();
            MoveXToUsteps(_xAxisConfig.ConvertRealUnitsToUstep(absMm));
            if (blocking)
            {
                WaitTillOperationIsCompleted(
                    CalcMoveTimeout(absMm - currentPos.XMm, _xAxisConfig.MaxSpeed));
            }
        }

        public void StageMoveYTo(double absMm, bool blocking = true)
        {
            var currentPos = StageGetPos();
            MoveYToUsteps(_yAxisConfig.ConvertRealUnitsToUstep(absMm));
            if (blocking)
            {
                WaitTillOperationIsCompleted(
                    CalcMoveTimeout(absMm - currentPos.YMm, _yAxisConfig.MaxSpeed));
            }
        }

        public void StageMoveZTo(double absMm, bool blocking = true)
        {
            var currentPos = StageGetPos();
            
            // Backlash compensation for Z axis
            bool needClearBacklash = absMm < currentPos.ZMm;

            if (blocking && needClearBacklash)
            {
                double backlashOffset = -BACKLASH_COMPENSATION_DISTANCE_MM;
                double clampedZBacklashPos = Math.Max(absMm + backlashOffset, _zAxisConfig.MinPosition);
                int clampedZBacklashPosUsteps = _zAxisConfig.ConvertRealUnitsToUstep(clampedZBacklashPos);
                
                MoveZToUsteps(clampedZBacklashPosUsteps);
                if (blocking)
                {
                    WaitTillOperationIsCompleted(
                        CalcMoveTimeout(clampedZBacklashPos - currentPos.ZMm, _zAxisConfig.MaxSpeed));
                }
            }

            MoveZToUsteps(_zAxisConfig.ConvertRealUnitsToUstep(absMm));
            if (blocking)
            {
                WaitTillOperationIsCompleted(
                    CalcMoveTimeout(absMm - currentPos.ZMm, _zAxisConfig.MaxSpeed));
            }
        }

        public StagePosition StageGetPos()
        {
            var posUsteps = GetPosition();
            return new StagePosition
            {
                XMm = _xAxisConfig.ConvertToRealUnits(posUsteps.X),
                YMm = _yAxisConfig.ConvertToRealUnits(posUsteps.Y),
                ZMm = _zAxisConfig.ConvertToRealUnits(posUsteps.Z),
                ThetaRad = _thetaAxisConfig.ConvertToRealUnits(posUsteps.Theta)
            };
        }

        public StageState StageGetState()
        {
            return new StageState { Busy = IsBusy() };
        }

        // Async version only for long-running home operation
        public async Task StageHomeAsync(bool x, bool y, bool z, bool theta, bool blocking = true)
        {
            // Calculate timeouts for each axis
            double xTimeout = CalcMoveTimeout(
                _xAxisConfig.MaxPosition - _xAxisConfig.MinPosition,
                _xAxisConfig.MaxSpeed / 5.0);
            double yTimeout = CalcMoveTimeout(
                _yAxisConfig.MaxPosition - _yAxisConfig.MinPosition,
                _yAxisConfig.MaxSpeed / 5.0);
            double zTimeout = CalcMoveTimeout(
                _zAxisConfig.MaxPosition - _zAxisConfig.MinPosition,
                _zAxisConfig.MaxSpeed / 5.0);
            double thetaTimeout = CalcMoveTimeout(2.0 * Math.PI, _thetaAxisConfig.MaxSpeed / 5.0);

            // Get homing directions using Python's logic: movement_sign_to_homing_direction(sign) = HomingDirection((sign + 1) / 2)
            // sign = 1  -> (1 + 1) / 2 = 1 -> Backward
            // sign = -1 -> (-1 + 1) / 2 = 0 -> Forward
            HomingDirection xDir = _xAxisConfig.MovementSign == 1 ? HomingDirection.Backward : HomingDirection.Forward;
            HomingDirection yDir = _yAxisConfig.MovementSign == 1 ? HomingDirection.Backward : HomingDirection.Forward;
            HomingDirection zDir = _zAxisConfig.MovementSign == 1 ? HomingDirection.Backward : HomingDirection.Forward;
            HomingDirection thetaDir = _thetaAxisConfig.MovementSign == 1 ? HomingDirection.Backward : HomingDirection.Forward;

            await Task.Run(() =>
            {
                if (x && y)
                {
                    HomeXY(xDir, yDir);
                }
                else if (x)
                {
                    HomeX(xDir);
                }
                else if (y)
                {
                    HomeY(yDir);
                }
                
                if (blocking && (x || y))
                {
                    WaitTillOperationIsCompleted(Math.Max(xTimeout, yTimeout));
                }

                if (z)
                {
                    HomeZ(zDir);
                    if (blocking)
                    {
                        WaitTillOperationIsCompleted(zTimeout);
                    }
                }

                if (theta)
                {
                    HomeTheta(thetaDir);
                    if (blocking)
                    {
                        WaitTillOperationIsCompleted(thetaTimeout);
                    }
                }
            });
        }

        public void StageZero(bool x, bool y, bool z, bool theta, bool blocking = true)
        {
            if (x)
            {
                ZeroX();
                if (blocking) WaitTillOperationIsCompleted();
            }

            if (y)
            {
                ZeroY();
                if (blocking) WaitTillOperationIsCompleted();
            }

            if (z)
            {
                ZeroZ();
                if (blocking) WaitTillOperationIsCompleted();
            }

            if (theta)
            {
                ZeroTheta();
                if (blocking) WaitTillOperationIsCompleted();
            }
        }

        public void StageSetLimits(
            double? xPosMm = null,
            double? xNegMm = null,
            double? yPosMm = null,
            double? yNegMm = null,
            double? zPosMm = null,
            double? zNegMm = null,
            double? thetaPosRad = null,
            double? thetaNegRad = null)
        {
            // Helper function to determine limit codes based on movement sign
            (byte neg, byte pos) LimitCodesFor(int movementSign, byte nonInvertedNeg, byte nonInvertedPos)
            {
                if (movementSign == 1)
                    return (nonInvertedNeg, nonInvertedPos);
                else if (movementSign == -1)
                    return (nonInvertedPos, nonInvertedNeg);
                else
                    throw new ArgumentException($"Only 1 and -1 are valid movement signs, but got: {movementSign}");
            }

            var (xNegCode, xPosCode) = LimitCodesFor(_xAxisConfig.MovementSign, LimitCode.X_NEGATIVE, LimitCode.X_POSITIVE);
            var (yNegCode, yPosCode) = LimitCodesFor(_yAxisConfig.MovementSign, LimitCode.Y_NEGATIVE, LimitCode.Y_POSITIVE);
            var (zNegCode, zPosCode) = LimitCodesFor(_zAxisConfig.MovementSign, LimitCode.Z_NEGATIVE, LimitCode.Z_POSITIVE);

            if (xPosMm.HasValue)
                SetLim(xPosCode, _xAxisConfig.ConvertRealUnitsToUstep(xPosMm.Value));

            if (xNegMm.HasValue)
                SetLim(xNegCode, _xAxisConfig.ConvertRealUnitsToUstep(xNegMm.Value));

            if (yPosMm.HasValue)
                SetLim(yPosCode, _yAxisConfig.ConvertRealUnitsToUstep(yPosMm.Value));

            if (yNegMm.HasValue)
                SetLim(yNegCode, _yAxisConfig.ConvertRealUnitsToUstep(yNegMm.Value));

            if (zPosMm.HasValue)
                SetLim(zPosCode, _zAxisConfig.ConvertRealUnitsToUstep(zPosMm.Value));

            if (zNegMm.HasValue)
                SetLim(zNegCode, _zAxisConfig.ConvertRealUnitsToUstep(zNegMm.Value));

            if (thetaNegRad.HasValue || thetaPosRad.HasValue)
                throw new InvalidOperationException("Setting limits for the theta axis is not supported on the CephlaStage");
        }

        // Provide async versions for UI scenarios
        public Task StageMoveXAsync(double relMm, bool blocking = true) => 
            Task.Run(() => StageMoveX(relMm, blocking));
        public Task StageMoveYAsync(double relMm, bool blocking = true) => 
            Task.Run(() => StageMoveY(relMm, blocking));
        public Task StageMoveZAsync(double relMm, bool blocking = true) => 
            Task.Run(() => StageMoveZ(relMm, blocking));
        public Task StageMoveXToAsync(double absMm, bool blocking = true) => 
            Task.Run(() => StageMoveXTo(absMm, blocking));
        public Task StageMoveYToAsync(double absMm, bool blocking = true) => 
            Task.Run(() => StageMoveYTo(absMm, blocking));
        public Task StageMoveZToAsync(double absMm, bool blocking = true) => 
            Task.Run(() => StageMoveZTo(absMm, blocking));
        public Task<StagePosition> StageGetPosAsync() => 
            Task.Run(() => StageGetPos());
        public Task<StageState> StageGetStateAsync() => 
            Task.Run(() => StageGetState());

        #endregion

        #region FilterWheel Methods

        public void FilterWheelInitialize(bool hasEncoderW = false)
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    if (_microcontroller == null) return;
                    
                    _filterWheelPositionIndex = FILTERWHEEL_MIN_INDEX;
                    
                    if (hasEncoderW && _defModule != null)
                    {
                        try
                        {
                            dynamic SQUID_FILTERWHEEL_MOTORSLOTINDEX = _defModule!.SQUID_FILTERWHEEL_MOTORSLOTINDEX;
                            dynamic PID_P_W = _defModule.PID_P_W;
                            dynamic PID_I_W = _defModule.PID_I_W;
                            dynamic PID_D_W = _defModule.PID_D_W;
                            dynamic SQUID_FILTERWHEEL_TRANSITIONS_PER_REVOLUTION = _defModule.SQUID_FILTERWHEEL_TRANSITIONS_PER_REVOLUTION;
                            dynamic ENCODER_FLIP_DIR_W = _defModule.ENCODER_FLIP_DIR_W;
                            dynamic ENABLE_PID_W = _defModule.ENABLE_PID_W;
                            
                            _microcontroller.set_pid_arguments(SQUID_FILTERWHEEL_MOTORSLOTINDEX, PID_P_W, PID_I_W, PID_D_W);
                            _microcontroller.configure_stage_pid(
                                SQUID_FILTERWHEEL_MOTORSLOTINDEX, 
                                SQUID_FILTERWHEEL_TRANSITIONS_PER_REVOLUTION, 
                                ENCODER_FLIP_DIR_W
                            );
                            _microcontroller.turn_on_stage_pid(SQUID_FILTERWHEEL_MOTORSLOTINDEX, ENABLE_PID_W);
                        }
                        catch (Exception ex)
                        {
                            Console.WriteLine($"Warning: Could not configure filter wheel encoder: {ex.Message}");
                        }
                    }
                }
            }
        }

        private void FilterWheelMoveW(double delta)
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    if (_microcontroller == null || _defModule == null) return;
                    
                    try
                    {
                        dynamic STAGE_MOVEMENT_SIGN_W = _defModule!.STAGE_MOVEMENT_SIGN_W;
                        dynamic SCREW_PITCH_W_MM = _defModule.SCREW_PITCH_W_MM;
                        dynamic MICROSTEPPING_DEFAULT_W = _defModule.MICROSTEPPING_DEFAULT_W;
                        dynamic FULLSTEPS_PER_REV_W = _defModule.FULLSTEPS_PER_REV_W;
                        
                        int usteps = (int)(STAGE_MOVEMENT_SIGN_W * delta / 
                            (SCREW_PITCH_W_MM / (MICROSTEPPING_DEFAULT_W * FULLSTEPS_PER_REV_W)));
                        
                        _microcontroller!.move_w_usteps(usteps);
                    }
                    catch (Exception ex)
                    {
                        Console.WriteLine($"Warning: Could not move filter wheel: {ex.Message}");
                    }
                }
            }
        }

        // Async version for long-running homing operation
        public async Task FilterWheelHomingAsync()
        {
            await Task.Run(() =>
            {
                HomeW(HomingDirection.Forward);
                WaitTillOperationIsCompleted(15.0);
                
                // Move to offset position
                if (_defModule != null)
                {
                    using (Py.GIL())
                    {
                        dynamic SQUID_FILTERWHEEL_OFFSET = _defModule.SQUID_FILTERWHEEL_OFFSET;
                        if (SQUID_FILTERWHEEL_OFFSET != null)
                           FilterWheelMoveW((double)SQUID_FILTERWHEEL_OFFSET);
                    }
                }
                
                _filterWheelPositionIndex = FILTERWHEEL_MIN_INDEX;
            });
        }

        public void FilterWheelNextPosition()
        {
            if (_filterWheelPositionIndex < FILTERWHEEL_MAX_INDEX)
            {
                if (_defModule != null)
                {
                    try
                    {
                        using (Py.GIL())
                        {
                            dynamic SCREW_PITCH_W_MM = _defModule.SCREW_PITCH_W_MM;
                            double moveDistance = (double)SCREW_PITCH_W_MM / 
                                (FILTERWHEEL_MAX_INDEX - FILTERWHEEL_MIN_INDEX + 1);
                            
                            FilterWheelMoveW(moveDistance);
                        }
                        WaitTillOperationIsCompleted(5.0);
                        _filterWheelPositionIndex++;
                    }
                    catch (Exception ex)
                    {
                        Console.WriteLine($"Warning: Could not move to next filter position: {ex.Message}");
                    }
                }
            }
        }

        public void FilterWheelPreviousPosition()
        {
            if (_filterWheelPositionIndex > FILTERWHEEL_MIN_INDEX)
            {
                if (_defModule != null)
                {
                    try
                    {
                        using (Py.GIL())
                        {
                            dynamic SCREW_PITCH_W_MM = _defModule.SCREW_PITCH_W_MM;
                            double moveDistance = -(double)SCREW_PITCH_W_MM / 
                                (FILTERWHEEL_MAX_INDEX - FILTERWHEEL_MIN_INDEX + 1);
                            
                            FilterWheelMoveW(moveDistance);
                        }
                        WaitTillOperationIsCompleted(5.0);
                        _filterWheelPositionIndex--;
                    }
                    catch (Exception ex)
                    {
                        Console.WriteLine($"Warning: Could not move to previous filter position: {ex.Message}");
                    }
                }
            }
        }

        public void FilterWheelSetPosition(int position)
        {
            if (position >= FILTERWHEEL_MIN_INDEX && position <= FILTERWHEEL_MAX_INDEX)
            {
                if (position != _filterWheelPositionIndex)
                {
                    if (_defModule != null)
                    {
                        try
                        {
                            using (Py.GIL())
                            {
                                dynamic SCREW_PITCH_W_MM = _defModule.SCREW_PITCH_W_MM;
                                double moveDistance = (position - _filterWheelPositionIndex) * (double)SCREW_PITCH_W_MM / 
                                    (FILTERWHEEL_MAX_INDEX - FILTERWHEEL_MIN_INDEX + 1);
                                
                                FilterWheelMoveW(moveDistance);
                            }
                            WaitTillOperationIsCompleted(5.0);
                            _filterWheelPositionIndex = position;
                        }
                        catch (Exception ex)
                        {
                            Console.WriteLine($"Warning: Could not set filter position: {ex.Message}");
                        }
                    }
                }
            }
            else
            {
                throw new ArgumentOutOfRangeException(nameof(position), 
                    $"Position must be between {FILTERWHEEL_MIN_INDEX} and {FILTERWHEEL_MAX_INDEX}");
            }
        }

        public int FilterWheelGetPosition() => _filterWheelPositionIndex;

        // Provide async versions for UI scenarios
        public Task FilterWheelInitializeAsync(bool hasEncoderW = false) => 
            Task.Run(() => FilterWheelInitialize(hasEncoderW));
        public Task FilterWheelNextPositionAsync() => 
            Task.Run(() => FilterWheelNextPosition());
        public Task FilterWheelPreviousPositionAsync() => 
            Task.Run(() => FilterWheelPreviousPosition());
        public Task FilterWheelSetPositionAsync(int position) => 
            Task.Run(() => FilterWheelSetPosition(position));

        #endregion

        #region Movement Commands - Synchronous by default

        public void MoveXUsteps(int usteps)
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    _microcontroller?.move_x_usteps(usteps);
                }
            }
        }

        public void MoveYUsteps(int usteps)
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    _microcontroller?.move_y_usteps(usteps);
                }
            }
        }

        public void MoveZUsteps(int usteps)
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    _microcontroller?.move_z_usteps(usteps);
                }
            }
        }

        public void MoveThetaUsteps(int usteps)
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    _microcontroller?.move_theta_usteps(usteps);
                }
            }
        }

        public void MoveWUsteps(int usteps)
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    _microcontroller?.move_w_usteps(usteps);
                }
            }
        }

        public void MoveXToUsteps(int usteps)
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    _microcontroller?.move_x_to_usteps(usteps);
                }
            }
        }

        public void MoveYToUsteps(int usteps)
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    _microcontroller?.move_y_to_usteps(usteps);
                }
            }
        }

        public void MoveZToUsteps(int usteps)
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    _microcontroller?.move_z_to_usteps(usteps);
                }
            }
        }

        public void SetOffsetVelocityX(double velocity)
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    _microcontroller?.set_off_set_velocity_x(velocity);
                }
            }
        }

        public void SetOffsetVelocityY(double velocity)
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    _microcontroller?.set_off_set_velocity_y(velocity);
                }
            }
        }

        #endregion

        #region Homing Commands

        public void HomeX(HomingDirection direction = HomingDirection.Forward)
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    if (_homingDirectionEnum != null && _microcontroller != null)
                    {
                        dynamic pyDirection = _homingDirectionEnum((int)direction);
                        _microcontroller!.home_x(homing_direction: pyDirection);
                    }
                }
            }
        }

        public void HomeY(HomingDirection direction = HomingDirection.Forward)
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    if (_homingDirectionEnum != null && _microcontroller != null)
                    {
                        dynamic pyDirection = _homingDirectionEnum((int)direction);
                        _microcontroller!.home_y(homing_direction: pyDirection);
                    }
                }
            }
        }

        public void HomeZ(HomingDirection direction = HomingDirection.Forward)
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    if (_homingDirectionEnum != null && _microcontroller != null)
                    {
                        dynamic pyDirection = _homingDirectionEnum((int)direction);
                        _microcontroller!.home_z(homing_direction: pyDirection);
                    }
                }
            }
        }

        public void HomeTheta(HomingDirection direction = HomingDirection.Forward)
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    if (_homingDirectionEnum != null && _microcontroller != null)
                    {
                        dynamic pyDirection = _homingDirectionEnum((int)direction);
                        _microcontroller!.home_theta(homing_direction: pyDirection);
                    }
                }
            }
        }

        public void HomeW(HomingDirection direction = HomingDirection.Forward)
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    if (_homingDirectionEnum != null && _microcontroller != null)
                    {
                        dynamic pyDirection = _homingDirectionEnum((int)direction);
                        _microcontroller!.home_w(homing_direction: pyDirection);
                    }
                }
            }
        }

        public void HomeXY(HomingDirection directionX = HomingDirection.Forward, 
                          HomingDirection directionY = HomingDirection.Forward)
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    if (_homingDirectionEnum != null && _microcontroller != null)
                    {
                        dynamic pyDirectionX = _homingDirectionEnum((int)directionX);
                        dynamic pyDirectionY = _homingDirectionEnum((int)directionY);
                        _microcontroller!.home_xy(homing_direction_x: pyDirectionX, homing_direction_y: pyDirectionY);
                    }
                }
            }
        }

        #endregion

        #region Zero Commands

        public void ZeroX()
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    _microcontroller?.zero_x();
                }
            }
        }

        public void ZeroY()
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    _microcontroller?.zero_y();
                }
            }
        }

        public void ZeroZ()
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    _microcontroller?.zero_z();
                }
            }
        }

        public void ZeroW()
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    _microcontroller?.zero_w();
                }
            }
        }

        public void ZeroTheta()
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    _microcontroller?.zero_theta();
                }
            }
        }

        #endregion

        #region Illumination Control

        public void SetIllumination(byte source, double intensity)
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    _microcontroller?.set_illumination(source, intensity);
                }
            }
        }

        public void SetIlluminationLedMatrix(byte source, double r, double g, double b)
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    _microcontroller?.set_illumination_led_matrix(source, r, g, b);
                }
            }
        }

        public void TurnOnIllumination()
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    _microcontroller?.turn_on_illumination();
                }
            }
        }

        public void TurnOffIllumination()
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    _microcontroller?.turn_off_illumination();
                }
            }
        }

        // Async versions for UI scenarios
        public Task SetIlluminationAsync(byte source, double intensity) => 
            Task.Run(() => SetIllumination(source, intensity));
        public Task SetIlluminationLedMatrixAsync(byte source, double r, double g, double b) => 
            Task.Run(() => SetIlluminationLedMatrix(source, r, g, b));
        public Task TurnOnIlluminationAsync() => 
            Task.Run(() => TurnOnIllumination());
        public Task TurnOffIlluminationAsync() => 
            Task.Run(() => TurnOffIllumination());

        #endregion

        #region Laser Control

        public void TurnOnAFLaser()
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    _microcontroller?.turn_on_AF_laser();
                }
            }
        }

        public void TurnOffAFLaser()
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    _microcontroller?.turn_off_AF_laser();
                }
            }
        }

        // Async versions for UI scenarios
        public Task TurnOnAFLaserAsync() => Task.Run(() => TurnOnAFLaser());
        public Task TurnOffAFLaserAsync() => Task.Run(() => TurnOffAFLaser());

        #endregion

        #region Hardware Trigger and Control

        public void SendHardwareTrigger(bool controlIllumination = false, uint illuminationOnTimeUs = 0, 
                                       byte triggerOutputCh = 0)
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    _microcontroller?.send_hardware_trigger(
                        control_illumination: controlIllumination,
                        illumination_on_time_us: (int)illuminationOnTimeUs,
                        trigger_output_ch: triggerOutputCh
                    );
                }
            }
        }

        public void SetStrobeDelayUs(uint strobeDelayUs, byte cameraChannel = 0)
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    _microcontroller?.set_strobe_delay_us((int)strobeDelayUs, cameraChannel);
                }
            }
        }

        public void SetAxisEnableDisable(byte axis, byte status)
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    _microcontroller?.set_axis_enable_disable(axis, status);
                }
            }
        }

        public void InitFilterWheel()
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    _microcontroller?.init_filter_wheel();
                }
            }
        }

        #endregion

        #region Motor Configuration

        public void ConfigureMotorDriver(byte axis, int microstepping, ushort currentRms, double iHold)
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    _microcontroller?.configure_motor_driver(axis, microstepping, currentRms, iHold);
                }
            }
        }

        public void SetMaxVelocityAcceleration(byte axis, double velocity, double acceleration)
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    _microcontroller?.set_max_velocity_acceleration(axis, velocity, acceleration);
                }
            }
        }

        public void SetLeadscrewPitch(byte axis, double pitchMm)
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    _microcontroller?.set_leadscrew_pitch(axis, pitchMm);
                }
            }
        }

        #endregion

        #region PID Control

        public void ConfigureStagePid(byte axis, int transitionsPerRevolution, bool flipDirection = false)
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    _microcontroller?.configure_stage_pid(axis, transitionsPerRevolution, flipDirection);
                }
            }
        }

        public void TurnOnStagePid(byte axis)
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    _microcontroller?.turn_on_stage_pid(axis);
                }
            }
        }

        public void TurnOffStagePid(byte axis)
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    _microcontroller?.turn_off_stage_pid(axis);
                }
            }
        }

        public void TurnOffAllPid()
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    _microcontroller?.turn_off_all_pid();
                }
            }
        }

        public void SetPidArguments(byte axis, double pidP, double pidI, double pidD)
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    _microcontroller?.set_pid_arguments(axis, pidP, pidI, pidD);
                }
            }
        }

        #endregion

        #region Limits and Safety

        public void SetLim(byte limitCode, int usteps)
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    _microcontroller?.set_lim(limitCode, usteps);
                }
            }
        }

        public void SetLimitSwitchPolarity(byte axis, byte polarity)
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    _microcontroller?.set_limit_switch_polarity(axis, polarity);
                }
            }
        }

        public void SetHomeSafetyMargin(byte axis, int margin)
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    _microcontroller?.set_home_safety_margin(axis, margin);
                }
            }
        }

        #endregion

        #region DAC and Analog Control

        public void AnalogWriteOnboardDAC(byte dac, ushort value)
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    _microcontroller?.analog_write_onboard_DAC(dac, value);
                }
            }
        }

        public void SetPiezoUm(double zPiezoUm)
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    _microcontroller?.set_piezo_um(zPiezoUm);
                }
            }
        }

        public void ConfigureDac80508RefdivAndGain(byte div, byte gains)
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    _microcontroller?.configure_dac80508_refdiv_and_gain(div, gains);
                }
            }
        }

        public void SetDac80508ScalingFactorForIllumination(double factor)
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    _microcontroller?.set_dac80508_scaling_factor_for_illumination(factor);
                }
            }
        }

        #endregion

        #region Configuration Methods

        public void ConfigureActuators()
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    _microcontroller?.configure_actuators();
                }
            }
        }

        public void ConfigureSquidFilter()
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    _microcontroller?.configure_squidfilter();
                }
            }
        }

        #endregion

        #region System Control

        public void Reset()
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    _microcontroller?.reset();
                }
            }
        }

        public void InitializeDrivers()
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    _microcontroller?.initialize_drivers();
                }
            }
        }

        public void SetPinLevel(byte pin, byte level)
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    _microcontroller?.set_pin_level(pin, level);
                }
            }
        }

        #endregion

        #region Joystick and Events

        public void EnableJoystick(bool enabled)
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    _microcontroller?.enable_joystick(enabled);
                }
            }
        }

        public void AckJoystickButtonPressed()
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    _microcontroller?.ack_joystick_button_pressed();
                }
            }
        }

        public byte GetButtonAndSwitchState()
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    return _microcontroller?.get_button_and_switch_state() ?? 0;
                }
            }
        }

        #endregion

        #region Status and Control

        // This is genuinely long-running, so keep async version
        public async Task WaitTillOperationIsCompletedAsync(double timeoutSeconds = 5.0, CancellationToken ct = default)
        {
            await Task.Run(() =>
            {
                lock (_commandLock)
                {
                    using (Py.GIL())
                    {
                        _microcontroller?.wait_till_operation_is_completed(timeout_limit_s: timeoutSeconds);
                    }
                }
            }, ct);
        }

        public void WaitTillOperationIsCompleted(double timeoutSeconds = 5.0)
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    _microcontroller?.wait_till_operation_is_completed(timeout_limit_s: timeoutSeconds);
                }
            }
        }

        public bool IsBusy()
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    return _microcontroller?.is_busy() ?? false;
                }
            }
        }

        public Position GetPosition()
        {
            lock (_commandLock)
            {
                using (Py.GIL())
                {
                    if (_microcontroller == null)
                        return new Position { X = 0, Y = 0, Z = 0, Theta = 0 };
                        
                    dynamic result = _microcontroller.get_pos();
                    return new Position
                    {
                        X = (int)result[0],
                        Y = (int)result[1],
                        Z = (int)result[2],
                        Theta = (int)result[3]
                    };
                }
            }
        }

        #endregion

        #region Background Monitoring

        public async Task StartPositionMonitoringAsync(int intervalMs = 100, CancellationToken ct = default)
        {
            await Task.Run(async () =>
            {
                while (!ct.IsCancellationRequested)
                {
                    try
                    {
                        var position = StageGetPos();
                        PositionUpdated?.Invoke(this, new PositionUpdateEventArgs { StagePosition = position });
                        await Task.Delay(intervalMs, ct);
                    }
                    catch (Exception ex)
                    {
                        Console.WriteLine($"Position monitoring error: {ex.Message}");
                        await Task.Delay(1000, ct);
                    }
                }
            }, ct);
        }

        #endregion

        #region IDisposable

        public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }

        protected virtual void Dispose(bool disposing)
        {
            if (!_disposed)
            {
                if (disposing)
                {
                    using (Py.GIL())
                    {
                        try
                        {
                            _microcontroller?.close();
                            _microcontroller?.Dispose();
                            _serialDevice?.close();
                            _serialDevice?.Dispose();
                            _microcontrollerModule?.Dispose();
                        }
                        catch (Exception ex)
                        {
                            Console.WriteLine($"Error during disposal: {ex.Message}");
                        }
                    }
                }
                _disposed = true;
            }
        }

        #endregion
    }

    #region Supporting Classes and Enums

    public enum HomingDirection
    {
        Forward = 0,
        Backward = 1
    }

    public static class Axis
    {
        public const byte X = 0;
        public const byte Y = 1;
        public const byte Z = 2;
        public const byte THETA = 3;
        public const byte W = 4;
        public const byte XY = 5;
    }

    public static class LimitCode
    {
        public const byte X_POSITIVE = 0;
        public const byte X_NEGATIVE = 1;
        public const byte Y_POSITIVE = 2;
        public const byte Y_NEGATIVE = 3;
        public const byte Z_POSITIVE = 4;
        public const byte Z_NEGATIVE = 5;
    }

    public static class MCU_PINS
    {
        public const byte AF_LASER = 10;
    }

    public class Position
    {
        public int X { get; set; }
        public int Y { get; set; }
        public int Z { get; set; }
        public int Theta { get; set; }

        public override string ToString()
        {
            return $"X:{X}, Y:{Y}, Z:{Z}, Theta:{Theta}";
        }
    }

    public class StagePosition
    {
        public double XMm { get; set; }
        public double YMm { get; set; }
        public double ZMm { get; set; }
        public double ThetaRad { get; set; }

        public override string ToString()
        {
            return $"X:{XMm:F3}mm, Y:{YMm:F3}mm, Z:{ZMm:F3}mm, Theta:{ThetaRad:F3}rad";
        }
    }

    public class StageState
    {
        public bool Busy { get; set; }
    }

    public class PositionUpdateEventArgs : EventArgs
    {
        public StagePosition StagePosition { get; set; } = new StagePosition();
    }

    public class StageConfig
    {
        public StageAxisConfig XAxis { get; set; } = new StageAxisConfig();
        public StageAxisConfig YAxis { get; set; } = new StageAxisConfig();
        public StageAxisConfig ZAxis { get; set; } = new StageAxisConfig();
        public StageAxisConfig ThetaAxis { get; set; } = new StageAxisConfig();
    }

    public class StageAxisConfig
    {
        public double ScrewPitch { get; set; }
        public int MicroSteppingDefault { get; set; }
        public int FullStepsPerRev { get; set; }
        public int MovementSign { get; set; }
        public double MaxSpeed { get; set; }
        public double MinPosition { get; set; }
        public double MaxPosition { get; set; }
        public bool UseEncoder { get; set; }
        public double EncoderStepSize { get; set; }
        public int EncoderSign { get; set; }
        public bool FlipDirection { get; set; }
        public bool PidEnabled { get; set; }
        public double PidP { get; set; }
        public double PidI { get; set; }
        public double PidD { get; set; }

        public int ConvertRealUnitsToUstep(double realUnits)
        {
            if (UseEncoder)
            {
                // For encoder: this shouldn't normally be used, but if needed:
                return (int)Math.Round(realUnits / (MovementSign * EncoderStepSize * EncoderSign));
            }
            else
            {
                // For stepper: match Python formula exactly
                return (int)Math.Round(realUnits / (MovementSign * ScrewPitch / (MicroSteppingDefault * FullStepsPerRev)));
            }
        }

        public double ConvertToRealUnits(int usteps)
        {
            if (UseEncoder)
            {
                // For encoder: match Python formula exactly
                return usteps * MovementSign * EncoderStepSize * EncoderSign;
            }
            else
            {
                // For stepper: match Python formula exactly
                return usteps * MovementSign * ScrewPitch / (MicroSteppingDefault * FullStepsPerRev);
            }
        }

    public static StageAxisConfig FromPythonDef(dynamic defModule, string axisName)
    {
        try
        {
            var config = new StageAxisConfig();
        
            switch (axisName.ToUpper())
            {
                case "X":
                    config.MovementSign = (int)defModule.STAGE_MOVEMENT_SIGN_X;
                    config.UseEncoder = (bool)defModule.USE_ENCODER_X;
                    config.EncoderSign = (int)defModule.ENCODER_POS_SIGN_X;
                    config.EncoderStepSize = (double)defModule.ENCODER_STEP_SIZE_X_MM;
                    config.FullStepsPerRev = (int)defModule.FULLSTEPS_PER_REV_X;
                    config.ScrewPitch = (double)defModule.SCREW_PITCH_X_MM;
                    config.MicroSteppingDefault = (int)defModule.MICROSTEPPING_DEFAULT_X;
                    config.MaxSpeed = (double)defModule.MAX_VELOCITY_X_mm;
                    config.MinPosition = (double)defModule.SOFTWARE_POS_LIMIT.X_NEGATIVE;
                    config.MaxPosition = (double)defModule.SOFTWARE_POS_LIMIT.X_POSITIVE;
                    config.FlipDirection = (bool)defModule.ENCODER_FLIP_DIR_X;
                    config.PidEnabled = (bool)defModule.ENABLE_PID_X;
                    config.PidP = (double)defModule.PID_P_X;
                    config.PidI = (double)defModule.PID_I_X;
                    config.PidD = (double)defModule.PID_D_X;
                    break;
                
                case "Y":
                    config.MovementSign = (int)defModule.STAGE_MOVEMENT_SIGN_Y;
                    config.UseEncoder = (bool)defModule.USE_ENCODER_Y;
                    config.EncoderSign = (int)defModule.ENCODER_POS_SIGN_Y;
                    config.EncoderStepSize = (double)defModule.ENCODER_STEP_SIZE_Y_MM;
                    config.FullStepsPerRev = (int)defModule.FULLSTEPS_PER_REV_Y;
                    config.ScrewPitch = (double)defModule.SCREW_PITCH_Y_MM;
                    config.MicroSteppingDefault = (int)defModule.MICROSTEPPING_DEFAULT_Y;
                    config.MaxSpeed = (double)defModule.MAX_VELOCITY_Y_mm;
                    config.MinPosition = (double)defModule.SOFTWARE_POS_LIMIT.Y_NEGATIVE;
                    config.MaxPosition = (double)defModule.SOFTWARE_POS_LIMIT.Y_POSITIVE;
                    config.FlipDirection = (bool)defModule.ENCODER_FLIP_DIR_Y;
                    config.PidEnabled = (bool)defModule.ENABLE_PID_Y;
                    config.PidP = (double)defModule.PID_P_Y;
                    config.PidI = (double)defModule.PID_I_Y;
                    config.PidD = (double)defModule.PID_D_Y;
                    break;
                
                case "Z":
                    config.MovementSign = (int)defModule.STAGE_MOVEMENT_SIGN_Z;
                    config.UseEncoder = (bool)defModule.USE_ENCODER_Z;
                    config.EncoderSign = (int)defModule.ENCODER_POS_SIGN_Z;
                    config.EncoderStepSize = (double)defModule.ENCODER_STEP_SIZE_Z_MM;
                    config.FullStepsPerRev = (int)defModule.FULLSTEPS_PER_REV_Z;
                    config.ScrewPitch = (double)defModule.SCREW_PITCH_Z_MM;
                    config.MicroSteppingDefault = (int)defModule.MICROSTEPPING_DEFAULT_Z;
                    config.MaxSpeed = (double)defModule.MAX_VELOCITY_Z_mm;
                    config.MinPosition = (double)defModule.SOFTWARE_POS_LIMIT.Z_NEGATIVE;
                    config.MaxPosition = (double)defModule.SOFTWARE_POS_LIMIT.Z_POSITIVE;
                    config.FlipDirection = (bool)defModule.ENCODER_FLIP_DIR_Z;
                    config.PidEnabled = (bool)defModule.ENABLE_PID_Z;
                    config.PidP = (double)defModule.PID_P_Z;
                    config.PidI = (double)defModule.PID_I_Z;
                    config.PidD = (double)defModule.PID_D_Z;
                    break;
                
                case "THETA":
                    config.MovementSign = (int)defModule.STAGE_MOVEMENT_SIGN_THETA;
                    config.UseEncoder = (bool)defModule.USE_ENCODER_THETA;
                    config.EncoderSign = (int)defModule.ENCODER_POS_SIGN_THETA;
                    config.EncoderStepSize = (double)defModule.ENCODER_STEP_SIZE_THETA;
                    config.FullStepsPerRev = (int)defModule.FULLSTEPS_PER_REV_THETA;
                    config.ScrewPitch = 2.0 * Math.PI / (int)defModule.FULLSTEPS_PER_REV_THETA;
                    config.MicroSteppingDefault = (int)defModule.MICROSTEPPING_DEFAULT_THETA;
                    config.MaxSpeed = 2.0 * Math.PI / 4;
                    config.MinPosition = 0;
                    config.MaxPosition = 2.0 * Math.PI / 4;
                    config.FlipDirection = false;
                    config.PidEnabled = false;
                    config.PidP = 1.0;
                    config.PidI = 0.0;
                    config.PidD = 0.0;
                    break;
                
                default:
                    throw new ArgumentException($"Unknown axis name: {axisName}");
            }
        
            return config;
        }
        catch (Exception ex)
        {
            throw new InvalidOperationException($"Failed to load config from Python _def for axis {axisName}: {ex.Message}", ex);
        }
    }
    }

    #endregion
}
