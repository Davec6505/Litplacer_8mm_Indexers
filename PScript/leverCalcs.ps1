function Get-TopLeverDisplacement {
    param (
        [double]$Ltop = 30.0,          # Radius to top contact point (mm)
        [double]$r = 36.0,             # Radius to bottom tracking point (mm)
        [double]$phiDeg = 45.0,        # Angle to bottom point in degrees
        [double]$theta0Deg = 0.0,      # Initial angle in degrees
        [double]$deltaX = 5.0          # Desired horizontal movement of bottom point (mm)
    )

    # Convert degrees to radians
    $phi = $phiDeg * [Math]::PI / 180
    $theta0 = $theta0Deg * [Math]::PI / 180

    # Compute initial bottom x-position
    $x0 = $r * [Math]::Cos($phi + $theta0)

    # Target x-position
    $xTarget = $x0 + $deltaX

    # Solve for new angle theta
    $cosTheta = $xTarget / $r
    if ($cosTheta -lt -1 -or $cosTheta -gt 1) {
        Write-Error "Requested deltaX is too large for given geometry."
        return
    }

    $thetaNew = [Math]::Acos($cosTheta) - $phi

    # Compute top lever downward displacement
    $y0 = $Ltop * [Math]::Sin($theta0)
    $yNew = $Ltop * [Math]::Sin($thetaNew)
    $deltaY = $yNew - $y0

    # Output results
    [PSCustomObject]@{
        Theta0_deg = [Math]::Round($theta0Deg, 2)
        ThetaNew_deg = [Math]::Round($thetaNew * 180 / [Math]::PI, 2)
        BottomX_Initial = [Math]::Round($x0, 3)
        BottomX_Target = [Math]::Round($xTarget, 3)
        TopY_Initial = [Math]::Round($y0, 3)
        TopY_New = [Math]::Round($yNew, 3)
        TopLever_Downward = [Math]::Round($deltaY, 3)
    }
}

# Example usage
Get-TopLeverDisplacement -Ltop 30 -r 36 -phiDeg 45 -theta0Deg 0 -deltaX 2
