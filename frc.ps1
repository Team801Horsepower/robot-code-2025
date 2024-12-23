# This script helps us complete common actions easily.

param (
    [Parameter(Position=0)]
    [string]$Action,

    [Parameter(Position=1)]
    [string]$Param
)

switch ($Action) {
    "verify" {
        Write-Host "Run mypy static type checker:" -ForegroundColor Green
        Write-Host "$("_" * $([console]::WindowWidth))"
        poetry run mypy src/

        Write-Host "`nRun ruff check:" -ForegroundColor Green
        Write-Host "$("_" * $([console]::WindowWidth))"
        poetry run ruff check --fix .
        poetry run ruff format .
    }
    "deploy" {
        poetry run robotpy --main src/robot.py deploy --skip-tests --team 801
    }
    "console" {
        poetry run netconsole 10.8.1.2
    }
    "shell" {
        poetry shell
    }
    "ssh" {
        ssh -o StrictHostKeyChecking=no admin@10.8.1.2
    }
    "rlds" {
        ssh -o StrictHostKeyChecking=no admin@10.8.1.2 "sh /home/lvuser/rlds/deploy"
    }
    "sb" {
        Get-ChildItem -Path "dev-utils\soundboard\" -Filter "$Param*.wav" | ForEach-Object {
            poetry run python dev-utils/soundboard/play.py $_.FullName | Out-Null
        }
    }
    "install" {
        poetry env remove --all
        poetry install --no-root
    }
    Default {
        Write-Host "Usage: ./frc.ps1 [verify | deploy | console | shell | ssh | rlds | sb | install] <optional param>" -ForegroundColor Red
    }
}

