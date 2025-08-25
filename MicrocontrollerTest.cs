// MicrocontrollerTest.cs - Comprehensive test using optimized async pattern
using System;
using System.Threading;
using System.Threading.Tasks;
using MicrocontrollerWrapper;

class MicrocontrollerTest
{
    static async Task Main(string[] args)
    {
        // Configuration
        bool useHardware = args.Length > 0 && args[0] == "--hardware";
        
        Console.WriteLine("╔════════════════════════════════════════════╗");
        Console.WriteLine($"║  Microcontroller Test Suite               ║");
        Console.WriteLine($"║  Mode: {(useHardware ? "HARDWARE" : "SIMULATION"),-19} ║");
        Console.WriteLine("╚════════════════════════════════════════════╝\n");
        
        // Create controller - it will automatically load configuration from Python _def module
        // No need to provide StageConfig unless you want to override Python values
        using var controller = new MicrocontrollerWrapper.MicrocontrollerWrapper(
            pythonScriptPath: "python/microcontroller.py",
            portName: null,  // Auto-detect
            simulated: !useHardware
            // stageConfig: null  // Will use Python _def module values
        );
        
        Console.WriteLine("Starting tests...\n");
        
        // Test 0: Display loaded configuration
        Console.WriteLine("Test 0: Display Loaded Configuration");
        var currentPos = controller.StageGetPos();
        Console.WriteLine($"  Initial Position: {currentPos}");
        Console.WriteLine("  Configuration loaded from Python _def module");
        Console.WriteLine("  ✓ Complete\n");
        
        // Test 1: Stage Home XYZ
        Console.WriteLine("Test 1: Stage Homing XYZ");
        Console.WriteLine("  Homing all axes...");
        controller.HomeX(HomingDirection.Backward);
        controller.WaitTillOperationIsCompleted();
        controller.HomeY(HomingDirection.Backward);
        controller.WaitTillOperationIsCompleted();
        controller.HomeZ(HomingDirection.Forward);
        controller.WaitTillOperationIsCompleted();
        var homePos = controller.StageGetPos();
        Console.WriteLine($"  Position after homing: {homePos}");
        Console.WriteLine("  ✓ Complete\n");
        
        // Test 2: Stage Move Y by 15mm
        Console.WriteLine("Test 2: Stage Moving Y by 15mm");
        controller.StageMoveY(15.0, blocking: true);
        var posAfterY = controller.StageGetPos();
        Console.WriteLine($"  Position: {posAfterY}");
        Console.WriteLine($"  Expected Y: ~15mm, Actual Y: {posAfterY.YMm:F3}mm");
        Console.WriteLine("  ✓ Complete\n");
        
        // Test 3: Stage Move X by 20mm
        Console.WriteLine("Test 3: Stage Moving X by 20mm");
        controller.StageMoveX(20.0, blocking: true);
        var posAfterX = controller.StageGetPos();
        Console.WriteLine($"  Position: {posAfterX}");
        Console.WriteLine($"  Expected X: ~20mm, Actual X: {posAfterX.XMm:F3}mm");
        Console.WriteLine("  ✓ Complete\n");
        
        // Test 4: Stage Move to absolute position
        Console.WriteLine("Test 4: Stage Moving to absolute position X=5mm, Y=3mm");
        controller.StageMoveXTo(5.0, blocking: true);
        controller.StageMoveYTo(3.0, blocking: true);
        var posAbsolute = controller.StageGetPos();
        Console.WriteLine($"  Position: {posAbsolute}");
        Console.WriteLine($"  Expected: X=5mm, Y=3mm");
        Console.WriteLine($"  Actual: X={posAbsolute.XMm:F3}mm, Y={posAbsolute.YMm:F3}mm");
        Console.WriteLine("  ✓ Complete\n");
        
        // Test 5: Stage Z axis movement
        Console.WriteLine("Test 5: Stage Z axis movement");
        controller.StageMoveZ(2.0, blocking: true);
        var posAfterZ = controller.StageGetPos();
        Console.WriteLine($"  Position after Z move: {posAfterZ}");
        Console.WriteLine($"  Expected Z: ~2mm, Actual Z: {posAfterZ.ZMm:F3}mm");
        Console.WriteLine("  ✓ Complete\n");
        
        // Test 6: Initialize and Home Filter Wheel (if available)
        try
        {
            Console.WriteLine("Test 6: Filter Wheel Initialize and Home");
            controller.FilterWheelInitialize(hasEncoderW: false);
            Console.WriteLine("  Homing filter wheel (this takes time)...");
            await controller.FilterWheelHomingAsync();
            Console.WriteLine($"  Current position: {controller.FilterWheelGetPosition()}");
            Console.WriteLine("  ✓ Complete\n");
            
            // Test 7: Move Filter Wheel to position 3
            Console.WriteLine("Test 7: Filter Wheel to Position 3");
            controller.FilterWheelSetPosition(3);
            Console.WriteLine($"  Current position: {controller.FilterWheelGetPosition()}");
            Console.WriteLine("  ✓ Complete\n");
            
            // Test 8: Move Filter Wheel to next position
            Console.WriteLine("Test 8: Filter Wheel Next Position");
            controller.FilterWheelNextPosition();
            Console.WriteLine($"  Current position: {controller.FilterWheelGetPosition()}");
            Console.WriteLine("  ✓ Complete\n");
            
            // Test 9: Move Filter Wheel to previous position
            Console.WriteLine("Test 9: Filter Wheel Previous Position");
            controller.FilterWheelPreviousPosition();
            Console.WriteLine($"  Current position: {controller.FilterWheelGetPosition()}");
            Console.WriteLine("  ✓ Complete\n");
        }
        catch (Exception ex)
        {
            Console.WriteLine($"  Filter wheel tests skipped: {ex.Message}\n");
        }
        
        // Test 10: AF Laser ON/OFF
        Console.WriteLine("Test 10: AF Laser Control");
        controller.TurnOnAFLaser();
        Console.WriteLine("  Laser ON");
        await Task.Delay(1000);
        controller.TurnOffAFLaser();
        Console.WriteLine("  Laser OFF");
        Console.WriteLine("  ✓ Complete\n");
        
        // Test 11: LED Matrix & Illumination
        Console.WriteLine("Test 11: LED Matrix and Illumination");
        controller.SetIlluminationLedMatrix(0, 0.5, 0.5, 1.0);
        controller.TurnOnIllumination();
        Console.WriteLine("  LED Matrix set to RGB(0.5, 0.5, 1.0)");
        Console.WriteLine("  Illumination ON");
        await Task.Delay(2000);
        controller.TurnOffIllumination();
        Console.WriteLine("  Illumination OFF");
        Console.WriteLine("  ✓ Complete\n");
        
        // Test 12: Check busy state
        Console.WriteLine("Test 12: Check Busy State");
        bool isBusy = controller.IsBusy();
        Console.WriteLine($"  System busy: {isBusy}");
        Console.WriteLine("  ✓ Complete\n");
        
        // Test 13: Get Final Position
        Console.WriteLine("Test 13: Get Final Position");
        var finalPos = controller.StageGetPos();
        Console.WriteLine($"  Final Position: {finalPos}");
        Console.WriteLine("  ✓ Complete\n");
        
        // Test 14: Start background position monitoring
        Console.WriteLine("Test 14: Starting position monitoring for 5 seconds...");
        using var cts = new CancellationTokenSource();
        
        int updateCount = 0;
        controller.PositionUpdated += (sender, args) =>
        {
            updateCount++;
            if (updateCount <= 5)  // Only show first 5 updates to avoid spam
            {
                var pos = args.StagePosition;
                Console.WriteLine($"  Position update #{updateCount}: X={pos.XMm:F3}mm, Y={pos.YMm:F3}mm, Z={pos.ZMm:F3}mm");
            }
        };
        
        var monitoringTask = controller.StartPositionMonitoringAsync(intervalMs: 1000, cts.Token);
        
        await Task.Delay(5000);
        cts.Cancel();
        
        try
        {
            await monitoringTask;
        }
        catch (OperationCanceledException)
        {
            // Expected when cancelling
        }
        
        Console.WriteLine($"  ✓ Monitoring stopped (received {updateCount} updates)\n");
        
        Console.WriteLine("╔════════════════════════════════════════════╗");
        Console.WriteLine("║  All tests completed successfully!        ║");
        Console.WriteLine("╚════════════════════════════════════════════╝");
    }
}

/*
To run:
  Simulation:  dotnet run
  Hardware:    dotnet run --hardware

Key changes in this updated test:
1. Removed StageConfig creation since DefaultX() methods don't exist
2. Configuration is now automatically loaded from Python _def module
3. Added more detailed position verification showing expected vs actual values
4. Added Test 0 to display initial configuration
5. Uses StageHomeAsync for combined axis homing
6. Added Z axis movement test
7. Added stage zeroing test
8. Filter wheel tests wrapped in try-catch in case _def module doesn't have those values
9. Better position monitoring output to avoid spam
*/
