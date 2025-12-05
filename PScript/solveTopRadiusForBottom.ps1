<#
.SYNOPSIS
Solves for the top lever radius (Ltop) needed to achieve a target multiple of bottom horizontal displacement.

.DESCRIPTION
Keeps the bottom geometry fixed (r, phiDeg, theta0Deg) and assumes the applied TOP vertical displacement (DeltaYTop)
is constant (e.g., imposed by a cam). If DeltaYTop is not provided, it is inferred from a baseline bottom displacement
(DeltaXBaseline) using the initial top radius (LtopInitial).

.PARAMETER r
Bottom radius to the tracking point (mm). Default: 36.

.PARAMETER phiDeg
Bottom point angular offset in degrees. Default: 45.

.PARAMETER theta0Deg
Initial angle in degrees. Default: 0.

.PARAMETER LtopInitial
Initial top radius (mm) used for baseline. Default: 30.

.PARAMETER Multiplier
Desired multiple of the baseline bottom displacement. Default: 2.

.PARAMETER DeltaYTop
Applied vertical movement at the top (mm). If omitted, it is inferred from DeltaXBaseline and LtopInitial.

.PARAMETER DeltaXBaseline
Baseline bottom displacement (mm) used to infer DeltaYTop when DeltaYTop is not given. Default: 2.

.EXAMPLE
PS> .\solveTopRadiusForBottom.ps1 -DeltaYTop 2 -LtopInitial 30 -Multiplier 2
Solves Ltop to double the bottom displacement when the top is driven 2 mm.

.EXAMPLE
PS> .\solveTopRadiusForBottom.ps1 -DeltaXBaseline 2 -LtopInitial 30 -Multiplier 2
Infers the top motion from a 2 mm baseline bottom displacement with LtopInitial, then computes Ltop to achieve 4 mm.

.OUTPUTS
PSCustomObject with geometry, baseline/target displacements, and LtopTarget.
#>

param (
    [double]$r = 36.0,
    [double]$phiDeg = 45.0,
    [double]$theta0Deg = 0.0,
    [double]$LtopInitial = 30.0,
    [double]$Multiplier = 2.0,
    [Nullable[double]]$DeltaYTop = $null,
    [double]$DeltaXBaseline = 2.0
)

# Helper: choose arcsin branch close to theta0
function Select-ThetaFromSin {
    param(
        [double]$sinTheta,
        [double]$theta0
    )
    if ($sinTheta -lt -1.0 -or $sinTheta -gt 1.0) {
        throw "Invalid sin(theta)=$sinTheta; out of [-1,1]."
    }
    $t1 = [Math]::Asin($sinTheta) # in [-pi/2, pi/2]
    $t2 = [Math]::PI - $t1        # supplementary
    # Pick the one closer to theta0
    if ([Math]::Abs($t1 - $theta0) -le [Math]::Abs($t2 - $theta0)) { return $t1 } else { return $t2 }
}

# Compute baseline quantities with initial Ltop and provided/inferred DeltaYTop
$phi = $phiDeg * [Math]::PI / 180
$theta0 = $theta0Deg * [Math]::PI / 180
$x0 = $r * [Math]::Cos($phi + $theta0)
$y0_init = $LtopInitial * [Math]::Sin($theta0)

if ($DeltaYTop -eq $null) {
    # Infer DeltaYTop from a baseline bottom displacement using LtopInitial
    $xTarget = $x0 + $DeltaXBaseline
    $cosTheta = $xTarget / $r
    if ($cosTheta -lt -1 -or $cosTheta -gt 1) {
        throw "Baseline DeltaX=$DeltaXBaseline is too large for given geometry."
    }
    $thetaNew_base = [Math]::Acos($cosTheta) - $phi
    $yNew_base = $LtopInitial * [Math]::Sin($thetaNew_base)
    $DeltaYTop = $yNew_base - $y0_init
}

# Baseline bottom displacement computed from DeltaYTop and LtopInitial
$sinThetaNew_base = [Math]::Sin($theta0) + ($DeltaYTop / $LtopInitial)
$thetaNew_fromY = Select-ThetaFromSin -sinTheta $sinThetaNew_base -theta0 $theta0
$xNew_base = $r * [Math]::Cos($phi + $thetaNew_fromY)
$DeltaX_base = $xNew_base - $x0
$DeltaX_target = $Multiplier * $DeltaX_base

# Function: bottom displacement for a given L
function Get-DeltaXForL {
    param(
        [double]$L
    )
    if ($L -le 0) { return [double]::NaN }
    $sinThetaNew = [Math]::Sin($theta0) + ($DeltaYTop / $L)
    if ($sinThetaNew -lt -1.0 -or $sinThetaNew -gt 1.0) { return [double]::NaN }
    $thetaNew = Select-ThetaFromSin -sinTheta $sinThetaNew -theta0 $theta0
    $xNew = $r * [Math]::Cos($phi + $thetaNew)
    return $xNew - $x0
}

# Root function f(L) = DeltaX(L) - DeltaX_target
function f {
    param([double]$L)
    $dx = Get-DeltaXForL -L $L
    if ([double]::IsNaN($dx)) { return [double]::NaN }
    return $dx - $DeltaX_target
}

# Bracket a root around LtopInitial by expanding
$L_lo = [Math]::Max(1e-6, $LtopInitial / 10.0)
$L_hi = $LtopInitial * 10.0
$maxExpand = 40
$foundBracket = $false
for ($i=0; $i -lt $maxExpand; $i++) {
    $f_lo = f -L $L_lo
    $f_hi = f -L $L_hi
    if (-not [double]::IsNaN($f_lo) -and -not [double]::IsNaN($f_hi) -and ($f_lo -eq 0 -or $f_hi -eq 0 -or ($f_lo -lt 0 -and $f_hi -gt 0) -or ($f_lo -gt 0 -and $f_hi -lt 0))) {
        $foundBracket = $true; break
    }
    # expand
    $L_lo /= 2.0
    $L_hi *= 2.0
}

if (-not $foundBracket) {
    throw "Failed to bracket a solution for Ltop. Try different Multiplier or check DeltaYTop magnitude."
}

# Bisection solve
$tol = 1e-6
$maxIter = 80
for ($iter=0; $iter -lt $maxIter; $iter++) {
    $L_mid = 0.5 * ($L_lo + $L_hi)
    $f_lo = f -L $L_lo
    $f_mid = f -L $L_mid
    if ([double]::IsNaN($f_mid)) {
        # move away from invalid zone by shrinking towards hi
        $L_lo = $L_mid
        continue
    }
    if ([Math]::Abs($f_mid) -lt $tol -or [Math]::Abs($L_hi - $L_lo) -lt $tol) {
        $LtopTarget = $L_mid
        break
    }
    if (($f_lo -lt 0 -and $f_mid -gt 0) -or ($f_lo -gt 0 -and $f_mid -lt 0)) {
        $L_hi = $L_mid
    } else {
        $L_lo = $L_mid
    }
}
if (-not $LtopTarget) { $LtopTarget = 0.5 * ($L_lo + $L_hi) }

# Verify
$dx_new = Get-DeltaXForL -L $LtopTarget

[PSCustomObject]@{
    r = $r
    phiDeg = $phiDeg
    theta0Deg = $theta0Deg
    LtopInitial = [Math]::Round($LtopInitial, 3)
    DeltaYTop_mm = [Math]::Round($DeltaYTop, 3)
    BaselineBottomDisp_mm = [Math]::Round($DeltaX_base, 3)
    Multiplier = $Multiplier
    TargetBottomDisp_mm = [Math]::Round($DeltaX_target, 3)
    LtopTarget_mm = [Math]::Round($LtopTarget, 6)
    AchievedBottomDisp_mm = [Math]::Round($dx_new, 6)
}
