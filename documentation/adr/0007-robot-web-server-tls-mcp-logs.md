# ADR 0007: Robot web server — TLS from a constrained team CA, MCP and log management in Rust

## Status

Accepted.

- **Implemented outside this repository**, in meta-frc4646 `eb9e0a7` and `6752256`: the team CA (`make_team_ca.sh`, `install_team_ca.sh`), the server certificate and a robot's fallback CA (`make_tls_cert.sh`, `aos-tls.service`, run again by `change_hostname.sh`), and `trust_orin.py`.
- **Implemented here**, in `aos/network/robot_server`: HTTPS on 443 with the certificate from `/etc/aos/tls`, reloaded when it changes; the `www` pages; `/ws` proxied to `web_proxy`; `/foxglove` proxied to the Foxglove server; the trust page on port 80; and `GET /api`, each channel's latest message as JSON. `aos/network/www/proxy.ts` connects over `wss://` from an https page.
- **Not yet implemented:** MCP, `/logs`, and `wss://` in `frc/image_streamer`'s pages.
- **Deferred:** replacing `web_proxy`'s WebRTC (rawrtc) with Rust.

## Context

Students connect to the robot from their own laptops: the plotting and field pages `aos/network/web_proxy` serves (port 1180), the camera pages `frc/image_streamer` serves, and Foxglove through `aos/util/foxglove_websocket` (port 8765). All are plain `http://` and `ws://`.

That stopped being enough with the Foxglove web app. It is served over `https://`, and an https page may only open `wss://` connections, to servers whose certificate the browser trusts. Our own pages have the same problem once they're served over https. The Foxglove desktop app also refuses untrusted connections to anything but localhost.

The workarounds don't scale to a team. An ssh tunnel per laptop (`ssh -L 8765:localhost:8765`) has to be set up by every student, every session.

Getting a certificate browsers trust is constrained by where robots run:

- `.local` names, which mDNS gives us everywhere (`orin-1868-1.local`), can't get certificates from a public CA.
- A public name can get a Let's Encrypt certificate, but at events laptops have no internet, so nothing resolves it without a hosts-file entry on every laptop.
- A conventional private CA that laptops trust can impersonate any site to them, `google.com` included. Its key would have to be guarded, and robots are not where you guard keys.
- Robots are renamed and reflashed often. Students shouldn't have to trust anything again when that happens.

We also want two more things from the robot, behind the same front door:

- **An MCP server**, so an AI assistant can work with a running robot: list channels, read and watch messages, read configs, timing reports and starter status, and, when allowed, send messages or restart applications.
- **Log management** without ssh: list, download (as stored or as MCAP), delete, copy off the robot.

### Security posture

Physical access is the security boundary, deliberately. The robot has open ssh with a well-known password, and `pi` has passwordless sudo: anyone on the robot network can be root. Nothing here tries to be stronger than that, and nothing here adds authentication.

What it must not do is make things worse _off_ the robot: trusting a robot must not let anyone impersonate any other site to a student's laptop, even someone with root on that robot.

Spoofing one of our robots on the robot network isn't worth defending against: anyone there can log in to the real one. The CA key still stays out of public repositories, so impersonating our robots takes being on a network with a student's laptop _and_ having taken the key from a robot.

## Decision

1. **Robot certificates come from a team CA that can only vouch for a set of robots.** It covers one or more team numbers. Its name constraints permit only `orin-<team>-0` to `9.local` and `10.TE.AM.0/24` for each team, its extended key usage only TLS servers, and `pathlen:0` forbids sub-CAs.
2. **The CA's key is kept private and put on each robot after flashing**, not baked into the public image. On a robot it needs no more protection than anything else there.
3. **Each robot issues its own server certificate from the CA at boot**, so renaming a robot within the CA's teams, or reflashing it and reinstalling the CA, doesn't change what laptops trust.
4. **A robot without the team CA makes a CA of its own**, constrained to its own name and address, so it still works, trusted separately.
5. **Laptops trust the CA with a script that refuses anything but exactly those two shapes.**
6. **One Rust server, `robot_server`, is the robot's https front door**: static pages, `/ws` proxied to `web_proxy`, `/api`, `/mcp` and `/logs`, on 443.
7. **It runs tokio on its AOS event loop's thread**, as ADR 0008 describes, so handlers use fetchers and senders directly.
8. **Flatbuffer ↔ JSON stays in C++**, reached from Rust through the C API, not reimplemented.
9. **Each feature returns an `axum::Router`**, and the binary nests them under one listener. Any can move into its own process later.
10. **MCP write tools are off unless the robot's config turns them on**, and then only for an allow-list of channels. This guards against an assistant's mistakes, not against attackers.
11. **WebRTC stays in C++ `web_proxy`.** `robot_server` proxies its websocket, which only carries WebRTC setup; the data channels go straight between the browser and `web_proxy`, encrypted with DTLS.

## Design

### The team CA

`make_team_ca.sh LABEL OUTDIR TEAM...` makes it, once, on a trusted machine; for example `make_team_ca.sh frc1868 ~/robot-ca 1868 9968 9969`. It is named `<LABEL> team robot CA`, valid for 20 years, EC P-256:

```
basicConstraints = critical, CA:TRUE, pathlen:0
keyUsage         = critical, keyCertSign, cRLSign
extendedKeyUsage = serverAuth
nameConstraints  = critical,
                   permitted;DNS:orin-1868-0.local, ... permitted;DNS:orin-1868-9.local,
                   (ten names for each further team),
                   permitted;IP:10.18.68.0/255.255.255.0,
                   (a subnet for each further team),
                   excluded;IP:::/::
```

- **Names:** the ten names `change_hostname.sh` allows for each team. Name constraints can't put a wildcard inside a name, so they're listed. Not `orin-1.local` or `orin.local`, aliases other teams' robots answer to as well.
- **Addresses:** each team's `10.TE.AM.0/24`. A permitted IPv4 subtree rejects other IPv4 addresses; IPv6 is excluded outright.
- **Uses:** `extendedKeyUsage = serverAuth` stops the CA vouching for email or code-signing certificates, which name constraints don't cover.
- **`critical`** on both constraints, so software that doesn't understand them rejects the certificate rather than ignoring them.

`install_team_ca.sh pi@orin-1868-1.local OUTDIR` puts it on a robot after flashing, in `/etc/aos/tls` with a `team` marker file, and issues the server certificate.

### On the robot

`aos-tls.service` runs `/root/bin/make_tls_cert.sh` at every boot, before `aos.service`, and `change_hostname.sh` runs it after a rename. It keeps these in `/etc/aos/tls`:

| File | Contents | Access |
| --- | --- | --- |
| `ca.crt`, `ca.key` | The team CA, or the robot's own | `ca.key` root only |
| `team` or `name` | Which of the two; `name` records the hostname and address the robot's own CA was made for |  |
| `server.crt`, `server.key` | The server certificate, for `<hostname>.local` and eth0's address, valid 397 days | `server.key` readable by `pi` |
| `server-chain.crt` | `server.crt` followed by `ca.crt` |  |

- With the team CA, the script keeps the CA and reissues the server certificate when there is none, when it doesn't verify for the current name and address, or when it expires within 60 days (397 days is the longest macOS accepts). If the new name is outside the CA's teams, it fails rather than serve a certificate nobody trusts.
- Without one, it makes a CA for this robot alone, constrained to `<hostname>.local` and its address, and replaces it when either changes.

### Trusting the robots

`trust_orin.py orin-1868-1` (Python 3.8 or newer, standard library only; macOS, Linux and Windows):

1. Derives the address from the name: `orin-TTTT-N` is `10.TT.TT.(100+N)`.
2. Fetches the CA from the robot's TLS server, which sends it after its own certificate.
3. Reads the certificate's DER directly, not OpenSSL's text output, which differs on macOS.
4. Refuses it unless the basic constraints are critical and `CA:TRUE, pathlen:0`, the extended key usage is exactly TLS server, and the name constraints are critical, exclude all of IPv6, and permit exactly either the ten names and subnet of each team a team CA names (with the robot's team among them), or exactly the one name and address of a robot's own CA. A CA swapped in on the way can't be used beyond those robots either.
5. Shows the CA's fingerprint, and asks before trusting a team CA, listing its teams.
6. Installs it: the macOS login keychain, trusted for SSL only; on Linux the NSS databases Chrome, Foxglove and each Firefox profile use, trusted for TLS only; the Windows user root store. Firefox on macOS and Windows needs `security.enterprise_roots.enabled`.

`--check` only checks; `--remove` takes it out again.

A browser can't tell an untrusted certificate from an unreachable server. A page served over plain http can: it loaded, so the robot is reachable, and if a probe of https then fails, the certificate isn't trusted. `robot_server` serves that page on port 80 for every path. Once the probe works, it goes on to the same path over https; until then it says how to run `trust_orin.py` and to restart the browser afterwards (Chrome remembers a rejected certificate until then), and probes again every 3 s. A bare `orin-1868-1` is sent to `orin-1868-1.local`, the only name the certificate has.

### The server

```
                       +---------------- robot_server, one thread ----------------+
 browser / Foxglove    |  tokio + LocalSet          ShmEventLoop's Aio             |
 / MCP client --TLS--> |  axum + rustls                                            |
                       |    /           static pages                              |
                       |    /ws         -> web_proxy                              |
                       |    /foxglove   -> Foxglove server                        |
                       |    /api        fetchers, JSON from C++                   |
                       |    /mcp, /logs (later)                                   |
                       +----------------------------------------------------------+
```

A Rust binary, run by starter like any AOS application, at normal priority. It never runs real time and nothing real time waits on it. Starter runs it as root so it can listen on 443 and 80; it then becomes `--user` (`pi`) for good, before it reads anything or makes the event loop's shared memory.

#### Flatbuffers and JSON

AOS's C++ already converts flatbuffers to JSON with the schemas in the config. The C API exposes it, and Rust's `ChannelExt::message_to_json()` wraps it. `/api` and MCP both use it.

#### `/api`

GET requests for what is on the channels, in the shape MCP's read tools will have:

- `GET /api/channels`: every channel, with its type, size, frequency, and whether it can be fetched.
- `GET /api/latest?name=…&type=…`: a channel's latest message, with its send times, as JSON.

Channels whose messages can be over 256 KiB (images), or that are read by pinning (whose readers are counted), aren't fetched. AOS makes fetchers only before the event loop runs, so the server makes one for every fetchable channel at startup.

#### MCP

Served at `/mcp` over streamable HTTP, with rmcp, the official Rust MCP SDK.

- **Read tools, always on:** `list_nodes`, `list_channels`, `list_applications`, `get_config`, `fetch_latest(channel)`, `watch(channel, count | seconds)`, `timing_report(application)`, `starter_status`.
- **Write tools, off unless the config enables them:** `send(channel, json)` on an allow-list of channels; `start`, `stop` and `restart(application)` through starter.
- Large messages (images) come back as a summary and a link.

#### Logs

Under `/logs`: list logs with sizes, times and disk usage; download as stored, or as MCAP through `log_to_mcap`; delete, and copy elsewhere; later, MCP tools that query a log in simulation. Log serving runs at idle I/O priority and rate-limits downloads, so it can't stall the logger.

#### Foxglove

`wss://<robot>/foxglove` is proxied to the Foxglove server on localhost:8765, passing the `foxglove.websocket.v1` subprotocol through, so Foxglove gets the server's certificate with no change to `aos/util/foxglove_websocket`. It costs one more copy of each message, which is cheap now that Foxglove gets no images. Later, the Foxglove library's own TLS (`ServerOptions::useTls`) could replace the proxy.

### Phases

1. **TLS front door:** the certificates, the pages, `/ws`, `/foxglove`, the trust page, and `/api`. Done.
2. **Read-only MCP** and `/logs`.
3. **MCP writes**, behind the config switch, and log queries.
4. **Deferred:** WebRTC in Rust.

## Consequences

Positive:

- Students open the robot's pages and Foxglove in a browser over https, after trusting the team CA once per laptop.
- Works offline at events: names resolve over mDNS.
- The CA key needs little guarding. Taken, it impersonates the team's robots and nothing else.
- Renaming a robot within its teams, or reflashing it and reinstalling the CA, changes nothing on laptops.
- One port and one certificate for pages, `/api`, MCP and logs. A new feature is another router.
- The real-time side is untouched, and WebRTC data doesn't pass through the server.

Trade-offs:

- After every flash, someone has to run `install_team_ca.sh`, or the robot falls back to a CA of its own.
- The CA's teams are fixed when it's made. Adding a team means a new CA, trusted again on every laptop.
- Everything rests on browsers and operating systems enforcing name constraints on a CA the user installed. RFC 5280 lets software skip them on a trust anchor; current Chrome, Firefox, macOS, Windows and OpenSSL are believed to enforce them, but see Verification.
- No authentication. With physical access as the boundary that's intended, but MCP write tools can move the robot, so they're off by default.
- Firefox needs a setting changed on macOS and Windows.

## Alternatives considered

- **Public certificates (Let's Encrypt) on a public name.** No laptop setup online, but offline the name doesn't resolve without hosts entries on every laptop, and renewal needs a DNS API token.
- **One unconstrained team CA (mkcert style).** Simplest, but whoever holds its key can fool every laptop that trusts it about any site, so the key can't live on robots.
- **Only a CA of each robot's own.** No key to distribute, but laptops trust every robot separately, again after each rename or reflash. Kept as the fallback.
- **Committing the team CA's key publicly.** No step after flashing, but anyone on any network with a student's laptop could impersonate the robots.
- **A self-signed certificate per robot.** No reliance on name constraints, but macOS caps TLS certificates at 825 days, so laptops would trust again every two years.
- **stunnel in front of the existing servers.** No code, but no MCP or logs, and another copy of everything.
- **TLS in seasocks.** Seasocks has none; adding it is fiddly and still leaves MCP and logs to build.
- **The MCP server and log tools in C++.** Possible, but Rust has the pieces (axum, rustls, rmcp), and AOS builds Rust.
- **Two threads, AOS and tokio, joined by queues.** Two context switches per request, and no simulation. ADR 0008 runs them on one thread instead.

## Tried, and why not

What we built first and changed, or started and stopped:

- **Ports 8443 and 8080.** Every link carried a port, and the robot's name typed into a browser got nothing. The server moved to 443 and 80. Starter runs it as root, as it runs `irq_affinity`; it listens, then becomes `pi` for good before reading anything.
- **A trust page of links.** The plain-http page first only linked to the https pages or to `trust_orin.py`, so everyone clicked through every time. It now goes on by itself once the probe works.
- **Probing the bare name.** `orin-1868-1` resolves through DNS, but the certificate only names `orin-1868-1.local`, so the probe failed even on trusted laptops. The page sends bare names to `.local`.
- **Telling a bad certificate from inside the page.** A browser reports it as a generic connection error. The plain-http probe is the only reliable signal; having the server record TLS alerts is a possible addition.
- **Caching.** Browsers kept serving old pages after a deploy. Every response now says `Cache-Control: no-cache`, so they check first (a 304 if unchanged).
- **Reimplementing `/ws` and WebRTC in Rust.** `web_proxy`'s data channels are rawrtc, about a thousand lines of C++ with its own packetization and acks. Proxying its websocket costs nothing, so WebRTC stays in C++ for now.
- **Flatbuffer JSON in Rust.** Rust has no reflection-driven JSON writer. A `cxx` bridge was the first plan; the existing C API is how the rest of AOS's Rust reaches C++, so `/api` uses that.
- **Making `/api`'s fetchers on demand.** AOS refuses to make fetchers once the event loop runs, so the first request crashed the server. It makes one per fetchable channel at startup.
- **Looking channels up without a node.** In a configuration with nodes, that crashes. Lookups use the event loop's node.
- **Fetching every channel.** Image channels are megabytes, and channels read by pinning count their readers, so a web fetcher could take a reader slot an application needs. Both are refused.
- **Streaming everything over `web_proxy`'s data channels.** With no rate limit, a 60 Hz channel such as a camera's TargetMap costs about 0.5 Mbit/s a camera, too much of a robot link for a page that only shows a count. Such data is fetched with `/api` a few times a second instead. `web_proxy` now sends a new subscriber the latest message straight away, rather than waiting for the next.
- **aws-lc-rs**, rustls's default crypto, needs cmake and a large C build. rustls uses ring. ring's build script needed a patch to the `rules_rust` that WORKSPACE builds use.

## Verification

Done, with OpenSSL 3.5:

- A team CA for teams 1868, 9968 and 123 vouches for `orin-1868-5.local`, `orin-123-0.local`, `10.18.68.5` and `10.1.23.200`. Certificates its key signs for `google.com`, `orin-1234-1.local`, `orin.local`, `10.18.69.1`, `192.168.0.1` and `::1` fail ("permitted subtree violation").
- A robot's own CA vouches for `orin-1868-1.local` and `10.18.68.101` only. Email, code-signing and client certificates fail, and so does anything from a sub-CA.
- With the team CA, renaming to a team it covers keeps the CA and reissues the server certificate; renaming outside it fails and leaves both alone.
- `trust_orin.py` accepts both kinds of CA and refuses every variation it should; on Linux it installs into each NSS database.

Done on orin-1868-1, from Chrome on Linux:

- The pages load over https on 443, from the name and the address. The bare name is refused, as the constraints intend.
- Port 80 sends a computer that trusts the robot on to https, and explains how to trust it otherwise.
- Foxglove connects to `wss://orin-1868-1.local/foxglove`.
- Pages get WebRTC data from `web_proxy` through `/ws`.
- `/api/latest` returns the robot's constants and its cameras' TargetMaps as JSON, and refuses image channels.
- The running server's real, effective and saved IDs are all `pi`'s, with no capabilities.

Still to do, before students rely on it:

- Have the team CA sign a `google.com` certificate, serve it, and confirm Chrome, Safari, Edge and Firefox on macOS and Windows reject it.
- Run `trust_orin.py` on macOS and Windows.
- Run `install_team_ca.sh` against a robot whose image has `make_tls_cert.sh`.

## References

- RFC 5280, section 4.2.1.10 (name constraints).
- ADR 0008 (tokio on the event loop's thread).
- `aos/network/web_proxy.cc`, `aos/network/www/proxy.ts`, `aos/util/foxglove_websocket_lib.h`.
- meta-frc4646: `make_team_ca.sh`, `install_team_ca.sh`, `trust_orin.py`, `make_tls_cert.sh`, `aos-tls.service`, `docs/details.md`.
