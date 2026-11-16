# Code review

A template for changes to this firmware. It is ordered by what has actually gone
wrong here, so the early sections are the ones that cost a reboot or a quiet
wrong answer, not the ones that are easiest to tick.

Review means **running it**. A review that only reads the diff has missed every
item in §2.

## How to use it

- One reviewer, the whole change, before it is flashed. Reviewing after flashing
  means the review is a commentary on a deployed binary.
- Quote the section and item number in the finding (`§3.2`), so a comment says
  what class of problem it is and the next round can check it was addressed.
- A must-fix item is a must-fix. Anything tagged **JUDGEMENT** is the author's
  call, argued in the commit message.
- Fill in §9 at the end. An empty box is an unreviewed change.

## 1. Scope

1.1 Does the commit do one thing? Mixed commits hide their own risk: a bug fix
    with a refactor inside it cannot be reverted independently. **MUST-FIX**
1.2 Is the claim in the subject true of the whole diff? Read the subject, then
    the diff, and check nothing else came along. **MUST-FIX**
1.3 Is anything unrelated left in the tree (`git status`)? Build products and
    editor droppings do not belong in a commit about parsing.

## 2. It runs

2.1 Builds clean from the committed tree, not from a dirty one. **MUST-FIX**
2.2 Flashed and exercised on the board — the changed path, not just the boot
    banner. **MUST-FIX**
2.3 Every route, command or field the change touches was called at least once
    and answered what the code says it answers. **MUST-FIX**
2.4 Something that was working before still works: NTP replies, `/healthz`,
    `/gps/info`, the survey, the antenna bias. Name what was re-tested.
    **MUST-FIX**
2.5 Boot counters and uptime are flat after the exercise, so nothing
    panic-looped through the test. `gps_rescans` and the drop counters are flat
    where they should be.
2.6 After the last flash the reported version matches the commit — not
    `-dirty`, not an older build.

## 3. Resource limits — the board is 500 KB of heap, not a workstation

3.1 Stack: anything added inside a task can raise its frame. New library
    machinery (`std::format`, deep recursion, big locals) needs the task's stack
    checked and raised in `sdkconfig`, not discovered by a panic. **MUST-FIX**
3.2 Allocation in a critical section: `portENTER_CRITICAL` is a spinlock and is
    not recursive. Formatting or `std::string` work under it must be justified,
    and anything that allocates belongs outside. **MUST-FIX**
3.3 Image growth: is the size acceptable against the 1.7 MB slot, and is the
    free space stated in the commit when it moves by more than a few KB?
3.4 Buffers: every one is either sized from a constant that appears once or
    carries a comment saying which C API needs it.
3.5 Heap after the exercise (§2.5) is not falling.

## 4. Persistence and recovery

4.1 A setter that changes behaviour is paired with `save()`. The setting looking
    right is not the test — the value surviving a reboot is. **MUST-FIX**
4.2 A state that is restored at boot was proven restored by actually rebooting
    between setting and reading, not by inspection.
4.3 A feature that can desynchronise something has a way back that does not
    need a power cycle. If it does need one, say so in the commit.
4.4 OTA: the new image confirms itself, and a broken one still falls back.
4.5 A power cut is a normal event here. What does this change cost if the supply
    drops mid-way?

## 5. Protocol and parsing

5.1 A frame is accepted only after its checksum. Nothing is decoded from a
    guessed offset. **MUST-FIX**
5.2 The declared length is checked against the definition before any field is
    read; variable-length messages are clamped to what the payload holds.
5.3 Byte fields are read at their real width. A four-byte field read as two is a
    silent truncation, not a compile error. **MUST-FIX**
5.4 A stream reader survives a message split across reads — feed it a frame cut
    at every offset, not one whole frame.
5.5 Units are named in the code, not inferred from a neighbour: cm and mm can sit
    in the same message, and a variance is not an accuracy. **MUST-FIX**
5.6 An ACK is awaited before the thing it acknowledges is acted on, and the
    transmitter is drained before a rate or mode change.

## 6. Language

6.1 No `snprintf`/`sscanf`/`strto*` on data the code owns; `text::put` and
    `text::getList`. `printf` stays for `ESP_LOG`, which is the SDK's signature.
6.2 No fixed char buffer that is not at a C boundary or POD shared under a
    spinlock, and each of those says why.
6.3 `std::formatter` takes the value's type: `volatile` needs a copy, `uint8_t`
    is a number. **MUST-FIX**
6.4 Parsing is strict — trailing junk, empty fields and wrong field counts are
    errors, not defaults.
6.5 Comments say *why*. Where a plausible alternative was rejected, name it and
    the number that decided it.

## 7. Concurrency

7.1 Who else touches this? State shared between tasks is either under the mux,
    atomic, or volatile and single-word plus monotonic.
7.2 Two readers of one stream: does the new code take bytes another reader
    wanted? **MUST-FIX**
7.3 A debounce or restore timer is long enough that a bouncing client cannot
    thrash the receiver before the restore finishes.

## 8. Docs and tests

8.1 `README.md` and `docs/` say what the change makes true, and stop saying what
    it made false.
8.2 New behaviour with a host-testable shape has a test (`tools/`, `unittest`,
    the framing suite). New behaviour without one says why in the commit.
8.3 New endpoints, fields or commands are in the docs with their ranges.
8.4 The commit message carries the evidence: what was run, what it returned,
    what surprised.

## 9. Anti-checklist — deliberate here, do not "fix"

- Exceptions and RTTI are off. `std::format` terminating on a bad format string
  is accepted.
- `char[]` at `lwIP`, `inet_ntop`, `httpd_req_recv` and `esp_ota_write`
  boundaries.
- No authentication on HTTP or tcp/3333: trusted LAN, by design.
- The bridge's synthesised MON-COMMS reply: the receiver ignores the message and
  u-center needs it.

## 10. Sign-off

| | |
|---|---|
| Commit | |
| Reviewer | |
| §2 exercises run | |
| Boots / uptime / heap after | |
| Findings, by section | |
| **JUDGEMENT** calls accepted, and why | |

---

### Worked example

`3c0cdd9` "GPS: save the antenna delay so it outlives the boot". §4.1 asks
whether the setter is paired with `save()` — the first version of that commit
was not, and the value read back correctly until the next boot, at which point
the boot's TP5 re-assert put the pulse back on an uncompensated figure. The
setting looked right the whole time and only the reboot test (§4.2) found it.
