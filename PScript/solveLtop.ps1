param (
    [double]$r = 36.0,             # Radius to bottom tracking point (mm)
    [double]$phiDeg = 45.0,         # Angle to bottom point in degrees
    [double]$theta0Deg = 0.0,       # Initial angle in degrees
    [double]$deltaX = 2.0,          # Desired horizontal movement of bottom point (mm)
    [double]$LtopInitial = 30.0,    # Current top lever radius (mm)
    [double]$Multiplier = 2.0       # Desired multiple of current top displacement
)

function Get-TopLeverDisplacement {
    param (
        [double]$Ltop,
        [double]$r,
        [double]$phiDeg,
        [double]$theta0Deg,
        [double]$deltaX
    )
    $phi = $phiDeg * [Math]::PI / 180
    $theta0 = $theta0Deg * [Math]::PI / 180
    $x0 = $r * [Math]::Cos($phi + $theta0)
    $xTarget = $x0 + $deltaX
    $cosTheta = $xTarget / $r
    if ($cosTheta -lt -1 -or $cosTheta -gt 1) {
        throw "Requested deltaX is too large for given geometry."
    }
    $thetaNew = [Math]::Acos($cosTheta) - $phi
    $y0 = $Ltop * [Math]::Sin($theta0)
    $yNew = $Ltop * [Math]::Sin($thetaNew)
    return $yNew - $y0
}

# Compute current displacement with the initial Ltop
$currentDisp = Get-TopLeverDisplacement -Ltop $LtopInitial -r $r -phiDeg $phiDeg -theta0Deg $theta0Deg -deltaX $deltaX
$targetDisp = $Multiplier * $currentDisp

# Observation: deltaY scales linearly with Ltop in this model because thetaNew is independent of Ltop
# Therefore, to achieve targetDisp, choose LtopTarget = LtopInitial * Multiplier
$LtopTarget = $LtopInitial * $Multiplier
$newDisp = Get-TopLeverDisplacement -Ltop $LtopTarget -r $r -phiDeg $phiDeg -theta0Deg $theta0Deg -deltaX $deltaX

[PSCustomObject]@{
    r = $r
    phiDeg = $phiDeg
    theta0Deg = $theta0Deg
    deltaX = $deltaX
    LtopInitial = $LtopInitial
    CurrentTopDisp_mm = [Math]::Round($currentDisp, 3)
    Multiplier = $Multiplier
    LtopTarget = [Math]::Round($LtopTarget, 3)
    NewTopDisp_mm = [Math]::Round($newDisp, 3)
}
