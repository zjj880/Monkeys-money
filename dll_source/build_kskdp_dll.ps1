param([string]$ZigPath = 'zig')
$ErrorActionPreference = 'Stop'
$source = Join-Path $PSScriptRoot 'KSKDPExternalFunction.c'
$sdk = Join-Path $PSScriptRoot 'sdk'
$output = Join-Path (Split-Path $PSScriptRoot -Parent) 'KSKDPExternalFunction.dll'
& $ZigPath cc -target x86_64-windows-gnu -std=c11 -O2 -Wall -Wextra -Wno-cast-function-type-mismatch -Wno-dll-attribute-on-redeclaration -shared $source "-I$sdk" -lws2_32 -o $output
if ($LASTEXITCODE -ne 0) { throw 'DLL compilation failed.' }
Write-Output "Built: $output"
