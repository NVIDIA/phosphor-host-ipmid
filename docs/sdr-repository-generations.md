# Dynamic SDR repository generations

## Published directory

The dynamic provider builds owned, serialized SDR generations in a yielding
background coroutine. Get SDR, Repository Info, Device SDR Info and DCMI use
one published generation. Discovery prepares a pending replacement without
mutating published bytes. No mutex spans a command or D-Bus wait.

A client can read a header and body, or repeat an offset-zero whole-record read,
while sensor discovery continues. Record meaning, count and successor IDs stay
consistent within the protected generation. Publication is a shared-pointer
replacement; no new protocol or OEM command is introduced.

## Command-visible behavior

| Command | Behavior |
| --- | --- |
| Get SDR Repository Info | Reports published records, including Type 8 entity records, and published addition/removal timestamps. Free space is `0xffff` (unspecified). Overflow is not advertised. Timestamps can lag discovery until a pending generation publishes. |
| Get Device SDR Info | Reports published sensor counts by the encoded owner LUN, or all published SDR records for the repository-count operation. Event-only records use the actual sensor LUN. |
| Reserve SDR Repository | Cancels the previous reservation, publishes any pending generation, and issues a nonzero token for the new view/requester. |
| Get SDR | Returns bytes and successor from the same generation. Nonzero offsets require a valid token and requester; stale or wrong tokens return `0xc5`. Offset-zero reads ignore the supplied token for compatibility. Invalid record selectors return `0xcc`; invalid offsets return `0xc9`. |
| DCMI sensor directory | Returns SDR record IDs, not sensor numbers. Clients read those records to obtain the sensor number/LUN used by live commands. Only full temperature records participate. |
| Live sensor commands | Resolve stable sensor identities. An unknown or removed sensor returns `0xcb`, rather than the old generic `0xcc`. Live values continue updating during a protected SDR walk. |
| Set Sensor Reading | Uses current ValueMutability metadata for authorization; it does not require a preceding Get SDR. |
| Set Sensor Thresholds | Reports property-write failure. Successful writes invalidate descriptor content once per command, including partial success before a later failure. Content-only updates do not stamp repository addition/removal times. The property call uses the IPMI D-Bus timeout. |
| Get/Read FRU | Returns Busy until acquisition succeeds and when discovery changes during an outstanding read. A provider replacement clears old raw bytes and cancels/logs a queued write rather than sending stale data to the new provider. An earlier accepted write response does not prove persistence after provider loss. |

Before initial publication, SDR directory and DCMI commands return Node Busy
(`0xc0`); a valid empty generation is different from an unpublished one.
Get Sensor Reading and FRU reads do not carry an SDR reservation.

Record selector `0x0000` selects the first record; `0xffff` selects the last.
Only the response's next-record `0xffff` marks the end of traversal. Serialized
record-ID bytes are normalized to the record's published position.

## Read protection and publication

IPMI has no End SDR Walk command. The read lease is local implementation policy,
separate from the IPMI reservation; it is not a global discovery lock.

- A contiguous, sequential traversal completes after every record has been
  read in full. The final header alone, gaps and out-of-order reads do not
  complete it.
- Owner inactivity expires a lease after five seconds. Other clients cannot
  extend that idle deadline.
- Sixty seconds is the absolute lifetime even for continuous owner reads.
- Another Reserve cancels the old reservation and establishes a new lease.
- Publication cancels the old reservation before exposing changed record IDs.
- A single timer targets the next pending-publication deadline. There is no
  periodic wakeup without pending work. Replaced timer callbacks cannot clear
  a newer timer's state; timer errors are logged and expiry is reevaluated.

A LAN client can defer publication within these bounds. The bound is a maximum
staleness policy, not a promise that arbitrary unreserved clients never straddle
a publication. Real client handling of Busy, cancellation and abandoned walks
requires firmware compatibility testing.

## Acquisition and recovery

Builders run one at a time. A changed input revision discards a completed
acquisition and schedules another build after the 50-millisecond coalescing
interval. Expected input-change races are not error-level acquisition failures.
Fresh inputs advance an older pending backoff; cancellation of that old wait
cannot clear the replacement schedule. Updates during a running build are
coalesced into its completion path.

An unchanged acquisition has five total attempts, with requested retry delays
of 1, 2, 4 and 8 seconds. Actual elapsed time also includes D-Bus operations.
Exhaustion logs degraded state and waits for relevant inputs/provider changes.
Unexpected exceptions are contained, logged and await changed inputs rather
than escaping the event loop or retrying indefinitely. A successful build resets
the budget. The FRU acquisition helper rejects stale replies; the surrounding
builder owns retries and scheduling.

Repository input callbacks clear descriptor acquisitions through one shared
invalidation path. Redundant mapper/Entity Manager/custom-name match sets are
removed. Property callbacks inspect names and skip variant values; ordinary
Value/Alarm updates do not rebuild SDRs. D-Bus matches constrain standard
interfaces and object roots; contributing-provider owner events trigger recovery.
Decorator and metadata replies cannot repopulate an invalidated acquisition.

A successful provider fetch negatively caches absent sensor paths for its normal
refresh period; discovery invalidates that cache. Unreadable existing sensors
can reuse their last descriptor. Removed paths are pruned from that fallback.
An interface that successfully yields no SDR representation does not consume a
retry budget. Concatenated wire records are retained individually; incomplete
trailing records are rejected. Producers remove their own structure padding.

## Required and optional FRUs

`-Dsdr-require-fru=true` is the default. A required FRU acquisition failure
prevents a new generation from publishing; an older valid generation remains
available. If no generation exists, directory commands remain Busy after retry
exhaustion until provider/input recovery. Logs expose that condition. This
intentionally avoids declaring a complete repository without required FRUs.

Platforms where FRUs are optional can build with
`-Dsdr-require-fru=false`. Unavailable discovery then contributes zero FRU
locators, and sensors plus fixed/entity records can publish. A later provider
owner/object event rebuilds the directory. This option is unsuitable when boot
requires FRU-backed system identification. ID exhaustion and malformed locators
remain failures; the optional policy does not hide them. The legacy OEM count
ABI still uses a failure sentinel, which the builder rejects rather than using
it as a record count.

Successful directory acquisition is not proof that every required device has
already been discovered. A required initial FRU set needs a separate platform
readiness contract; generation consistency cannot manufacture missing inventory.

## Identity lifetime and compatibility

Sensor numbers allocate densely through LUN 0, then LUN 1, then LUN 3; `0xff`
and all of LUN 2 are excluded. A small initial directory therefore stays on
LUN 0. Late additions retain existing allocations. Allocation depends on first
discovery order and is not guaranteed identical across daemon restarts.

Physical FRU bus/address pairs retain their IDs. Initial preferred IDs follow
the legacy object-path hash and collision search to reduce upgrade renumbering;
later label changes do not redirect an allocated identity. FRU ID 0 remains
bound to the first physical baseboard for the daemon lifetime. A replacement
baseboard at a different bus/address uses a nonzero ID; FRU ID 0 is absent
while the original location is absent. Rediscovery at the original bus/address
restores ID 0. Identity here means a bus/address location, not an EEPROM serial
number: replacement hardware at the same location inherits that location's ID.
A topology change that requires a different location to become FRU ID 0 needs
an IPMI provider daemon restart and client rediscovery. A FruDevice provider
restart alone does not reset the ID lifetime. Automatic same-lifetime ID 0
reassignment is deliberately excluded because unreserved clients may retain it.

Removed identities retain bounded tombstones until the IPMI provider daemon
restarts. There is no timed reassignment: unreserved clients can cache IDs longer
than a lease. More than 765 distinct sensor identities or 254 non-baseboard FRU
identities can exhaust a daemon-lifetime pool. Rediscovery of the same identity
does not consume a slot. Exhaustion rejects a replacement generation and retains
the published one. Live sensor lookups retain and cache currently present paths
that already have allocations, so they do not repeatedly rebuild the mapper or
all fail because an unrelated new path could not be assigned. Removed objects
remain unavailable, and new unallocated paths are not advertised as complete.

Clients must rediscover IDs after an upgrade or daemon restart. Dense allocation
reduces LUN compatibility changes, but does not make an old cached mapping valid
across resets. Automatic reclamation without a client lifetime contract would
reintroduce misdirected live reads.

## Validation boundary

The reservation policy follows IPMI v2.0 sections 33.11.2 and 33.12/Table 33-6:
new reservations and record-ID-changing publication cancel an old reservation;
partial reads with nonzero offsets require a valid token. DCMI directory entries
are SDR record IDs, which are distinct from the sensor numbers encoded inside.

Pure repository and isolated handler tests do not establish boot-time SBIOS
Busy compatibility, real provider/write recovery, OEM extension behavior, wire
timing or load limits. Fresh firmware validation must check full SDR/FRU walks,
current-boot SMBIOS, concurrent clients, provider changes and actual discovery
overlap. Historical tests of an older image are not evidence for a revised one.
This design does not extend an SSIF deadline or repair its transport state
machine, and does not guarantee discovery completes before host boot.
