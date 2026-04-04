# Comprehensive Test Suite Runner
# Runs all unit tests and SIL simulations

Write-Host "`n╔════════════════════════════════════════════════════════════╗" -ForegroundColor Cyan
Write-Host "║        IGNITIA AVIONICS - COMPREHENSIVE TEST SUITE         ║" -ForegroundColor Cyan
Write-Host "╚════════════════════════════════════════════════════════════╝`n" -ForegroundColor Cyan

$totalTests = 0
$passedTests = 0
$failedTests = 0

# PART 1: UNIT TESTS
Write-Host "═══════════════════════════════════════════════════════════" -ForegroundColor Yellow
Write-Host "PART 1: UNIT TESTS" -ForegroundColor Yellow
Write-Host "═══════════════════════════════════════════════════════════`n" -ForegroundColor Yellow

# Test 1: Freefall Detection
Write-Host "Running: Freefall Detection Tests..." -ForegroundColor Cyan
Set-Location test
gcc -DHOST_TEST -I../src -I../src/modules -I../src/utils test_freefall_detection.c -o test_freefall_detection.exe 2>$null
if ($?) {
    $output = & .\test_freefall_detection.exe 2>&1 | Out-String
    if ($output -match "Results: (\d+)/(\d+) tests passed") {
        $passed = [int]$matches[1]
        $total = [int]$matches[2]
        $totalTests += $total
        $passedTests += $passed
        $failedTests += ($total - $passed)
        if ($passed -eq $total) {
            Write-Host "  ✓ Freefall Detection: $passed/$total PASSED" -ForegroundColor Green
        } else {
            Write-Host "  ✗ Freefall Detection: $passed/$total passed" -ForegroundColor Red
        }
    }
} else {
    Write-Host "  ✗ Compilation failed" -ForegroundColor Red
}

# Test 2: Apogee Detection
Write-Host "Running: Apogee Detection Tests..." -ForegroundColor Cyan
gcc -I../src -I../src/modules -I../src/utils test_apogee_detection.c -lm -o test_apogee_detection.exe 2>$null
if ($?) {
    $output = & .\test_apogee_detection.exe 2>&1 | Out-String
    if ($output -match "Passed: (\d+)") {
        $passed = [int]$matches[1]
        if ($output -match "Failed: (\d+)") {
            $failed = [int]$matches[1]
            $total = $passed + $failed
            $totalTests += $total
            $passedTests += $passed
            $failedTests += $failed
            if ($failed -eq 0) {
                Write-Host "  ✓ Apogee Detection: $passed/$total PASSED" -ForegroundColor Green
            } else {
                Write-Host "  ✗ Apogee Detection: $passed/$total passed" -ForegroundColor Red
            }
        }
    }
} else {
    Write-Host "  ✗ Compilation failed" -ForegroundColor Red
}

Set-Location ..

# PART 2: SIL SIMULATIONS
Write-Host "`n═══════════════════════════════════════════════════════════" -ForegroundColor Yellow
Write-Host "PART 2: SOFTWARE-IN-THE-LOOP (SIL) SIMULATIONS" -ForegroundColor Yellow
Write-Host "═══════════════════════════════════════════════════════════`n" -ForegroundColor Yellow

# Activate venv
& "venv\Scripts\Activate.ps1" 2>$null

# Create results directory
if (-not (Test-Path "sil/results")) {
    New-Item -ItemType Directory -Path "sil/results" | Out-Null
}

$silTests = @(
    @{Name="Nominal Flight (400m)"; File="nominal_flight.json"},
    @{Name="Early Apogee (200m)"; File="early_apogee.json"},
    @{Name="Sensor Failure"; File="sensor_failure.json"}
)

$silPassed = 0
$silFailed = 0

foreach ($test in $silTests) {
    Write-Host "Running: $($test.Name)..." -ForegroundColor Cyan
    $output = python sil/sil_runner.py "sil/test_scenarios/$($test.File)" -o "sil/results/$($test.File)" 2>&1 | Out-String
    
    if ($output -match "DEPLOYMENT at") {
        Write-Host "  ✓ $($test.Name): Deployment successful" -ForegroundColor Green
        $silPassed++
    } else {
        Write-Host "  ⚠ $($test.Name): Check results" -ForegroundColor Yellow
        $silFailed++
    }
}

# PART 3: FIRMWARE BUILD
Write-Host "`n═══════════════════════════════════════════════════════════" -ForegroundColor Yellow
Write-Host "PART 3: FIRMWARE BUILD VERIFICATION" -ForegroundColor Yellow
Write-Host "═══════════════════════════════════════════════════════════`n" -ForegroundColor Yellow

if (Test-Path "build/ignitia_avionics.uf2") {
    $uf2 = Get-Item "build/ignitia_avionics.uf2"
    Write-Host "  ✓ Firmware: ignitia_avionics.uf2 ($([math]::Round($uf2.Length/1KB,1)) KB)" -ForegroundColor Green
    Write-Host "  ✓ Last built: $($uf2.LastWriteTime)" -ForegroundColor Green
} else {
    Write-Host "  ✗ Firmware not found - run 'python build.py'" -ForegroundColor Red
}

# FINAL SUMMARY
Write-Host "`n╔════════════════════════════════════════════════════════════╗" -ForegroundColor Cyan
Write-Host "║                      FINAL SUMMARY                         ║" -ForegroundColor Cyan
Write-Host "╚════════════════════════════════════════════════════════════╝`n" -ForegroundColor Cyan

Write-Host "Unit Tests:" -ForegroundColor White
Write-Host "  Total: $totalTests tests" -ForegroundColor White
Write-Host "  Passed: $passedTests" -ForegroundColor Green
Write-Host "  Failed: $failedTests" -ForegroundColor $(if ($failedTests -eq 0) { "Green" } else { "Red" })

Write-Host "`nSIL Simulations:" -ForegroundColor White
Write-Host "  Total: $($silTests.Count) scenarios" -ForegroundColor White
Write-Host "  Passed: $silPassed" -ForegroundColor Green
Write-Host "  Failed: $silFailed" -ForegroundColor $(if ($silFailed -eq 0) { "Green" } else { "Red" })

$overallSuccess = ($failedTests -eq 0) -and ($silFailed -eq 0)
Write-Host "`nOverall Status: " -NoNewline -ForegroundColor White
if ($overallSuccess) {
    Write-Host "✓ ALL TESTS PASSED" -ForegroundColor Green
} else {
    Write-Host "✗ SOME TESTS FAILED" -ForegroundColor Red
}

Write-Host "`n════════════════════════════════════════════════════════════`n" -ForegroundColor Cyan
