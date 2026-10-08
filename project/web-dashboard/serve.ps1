param(
  [int]$Port = 4173
)

$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$listener = [System.Net.Sockets.TcpListener]::new(
  [System.Net.IPAddress]::Loopback,
  $Port
)

function Get-ContentType([string]$Path) {
  switch ([System.IO.Path]::GetExtension($Path).ToLowerInvariant()) {
    ".html" { return "text/html; charset=utf-8" }
    ".css"  { return "text/css; charset=utf-8" }
    ".js"   { return "application/javascript; charset=utf-8" }
    default { return "application/octet-stream" }
  }
}

$listener.Start()
try {
  while ($true) {
    $client = $listener.AcceptTcpClient()
    try {
      $stream = $client.GetStream()
      $reader = [System.IO.StreamReader]::new(
        $stream,
        [System.Text.Encoding]::ASCII,
        $false,
        1024,
        $true
      )
      $requestLine = $reader.ReadLine()
      while ($reader.ReadLine()) { }

      $requestPath = "/"
      if ($requestLine -match "^GET\s+([^\s]+)") {
        $requestPath = [System.Uri]::UnescapeDataString($Matches[1].Split("?")[0])
      }

      $relativePath = switch ($requestPath) {
        "/" { "index.html" }
        "/index.html" { "index.html" }
        "/styles.css" { "styles.css" }
        "/app.js" { "app.js" }
        default { $null }
      }

      if ($null -eq $relativePath) {
        $status = "404 Not Found"
        $contentType = "text/plain; charset=utf-8"
        $body = [System.Text.Encoding]::UTF8.GetBytes("Not found")
      }
      else {
        $status = "200 OK"
        $filePath = Join-Path $root $relativePath
        $contentType = Get-ContentType $filePath
        $body = [System.IO.File]::ReadAllBytes($filePath)
      }

      $header = "HTTP/1.1 $status`r`nContent-Type: $contentType`r`nContent-Length: $($body.Length)`r`nCache-Control: no-store`r`nConnection: close`r`n`r`n"
      $headerBytes = [System.Text.Encoding]::ASCII.GetBytes($header)
      $stream.Write($headerBytes, 0, $headerBytes.Length)
      $stream.Write($body, 0, $body.Length)
      $stream.Flush()
    }
    finally {
      $client.Dispose()
    }
  }
}
finally {
  $listener.Stop()
}
