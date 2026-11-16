# Code review

A template for changes to this firmware. Part A settles the decisions that are
expensive to reverse, before there is a diff. Part B is ordered by what has
actually gone wrong here, so its early sections are the ones that cost a reboot or
a quiet wrong answer, not the ones that are easiest to tick.

Review means **running it**. A review that only reads the diff has missed every
item in §2 — and §3 is what to do when the board cannot be spared for that.

## How to use it

- One reviewer, the whole change, before it is flashed. Reviewing after flashing
  means the review is a commentary on a deployed binary.
- Quote the section and item number in the finding (`§6.2`), so a comment says
  what class of problem it is and the next round can check it was addressed.
- A must-fix item is a must-fix. Anything tagged **JUDGEMENT** is the author's
  call, argued in the commit message.
- Fill in §11 at the end. An empty box is an unreviewed change.

---

# Part A — before the code exists

## 0. Design review

A paragraph in the commit message or a short `docs/` note, argued with the
rejected alternative and the number that decided it. Not a document. It happens
once per change, before the first line, and its output is what Part B reviews the
diff against.

0.1 **Shape.** Endpoint, field names, wire or NVS encoding, units. These are what
    a later change has to stay compatible with, so they are settled first and
    written down. **MUST-FIX**
0.2 **Reversibility.** If this ships wrong, what is the way back, and does it need
    physical access? A persisted format or a changed baud is expensive; a
    function name is not.
0.3 **Budget.** Stack, heap, image, flash writes, and which task pays. Numbers,
    not adjectives. **MUST-FIX**
0.4 **Failure story.** What does the code do when the receiver NAKs, the power
    drops mid-write, the client vanishes, the buffer is short? "Cannot happen" is
    an answer only with the reason attached.
0.5 **Blast radius.** What else reads this state, and what breaks while it is
    half-applied?
0.6 **Acceptance.** The exact exercise that will prove it, stated before the
    code, so §2 cannot quietly become "it booted".

A design that cannot fill 0.1–0.4 yet is not ready to be coded. Say so and stop.

---

# Part B — the diff

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

## 3. When the board cannot be exercised

The board serves time and may be mid-survey. A reboot costs the receiver's
almanac, a baud move costs the link until the scan finds it, and a cold start
costs the survey itself. Sometimes the change has to land first and be exercised
later — but only under these rules.

3.1 **What may be deferred:** nothing on the serving path (the anchor, the PPS
    handling, the NTP reply builder), nothing in OTA or rollback, nothing that
    touches a persisted format's *write* path. Those are exercised now or the
    change does not land.
3.2 **What must not be deferred:** §2.2 and §2.4. The rest of §2 may be.
    **MUST-FIX**
3.3 **Who decides:** the reviewer, on the author stating the cost — "this needs a
    cold start and there are nineteen hours of survey left".
3.4 **What is recorded:** the unrun check, why, and the trigger that will run it,
    in the commit message, in the same words the reviewer will look for later. A
    deferred check with no trigger is a forgotten one.
3.5 **The change stays revertible** until the deferred checks pass: no follow-up
    built on top of it, so a bad result can still mean one revert.
3.6 **The deferral expires.** It runs at the survey's end, or before the next
    flash, whichever comes first. Say which in the commit.
3.7 **A deferred check that then fails** is a fix, not a new feature, and it goes
    to the top of the next change.

## 4. Resource limits — the board is 500 KB of heap, not a workstation

4.1 Stack: anything added inside a task can raise its frame. New library
    machinery (`std::format`, deep recursion, big locals) needs the task's stack
    checked and raised in `sdkconfig`, not discovered by a panic. **MUST-FIX**
4.2 Allocation in a critical section: `portENTER_CRITICAL` is a spinlock and is
    not recursive. Formatting or `std::string` work under it must be justified,
    and anything that allocates belongs outside. **MUST-FIX**
4.3 Image growth: is the size acceptable against the 1.7 MB slot, and is the free
    space stated in the commit when it moves by more than a few KB?
4.4 Buffers: every one is either sized from a constant that appears once or
    carries a comment saying which C API needs it.
4.5 Heap after the exercise (§2.5) is not falling.

## 5. Persistence and recovery

5.1 A setter that changes behaviour is paired with `save()`. The setting looking
    right is not the test — the value surviving a reboot is. **MUST-FIX**
5.2 A state that is restored at boot was proven restored by actually rebooting
    between setting and reading, not by inspection.
5.3 A feature that can desynchronise something has a way back that does not
    need a power cycle. If it does need one, say so in the commit.
5.4 OTA: the new image confirms itself, and a broken one still falls back.
5.5 A power cut is a normal event here. What does this change cost if the supply
    drops mid-way?

## 6. Protocol and parsing

6.1 A frame is accepted only after its checksum. Nothing is decoded from a
    guessed offset. **MUST-FIX**
6.2 The declared length is checked against the definition before any field is
    read; variable-length messages are clamped to what the payload holds.
6.3 Byte fields are read at their real width. A four-byte field read as two is a
    silent truncation, not a compile error. **MUST-FIX**
6.4 A stream reader survives a message split across reads — feed it a frame cut
    at every offset, not one whole frame.
6.5 Units are named in the code, not inferred from a neighbour: cm and mm can sit
    in the same message, and a variance is not an accuracy. **MUST-FIX**
6.6 An ACK is awaited before the thing it acknowledges is acted on, and the
    transmitter is drained before a rate or mode change.

## 7. Language

7.1 No `snprintf`/`sscanf`/`strto*` on data the code owns; `text::put` and
    `text::getList`. `printf` stays for `ESP_LOG`, which is the SDK's signature.
7.2 No fixed char buffer that is not at a C boundary or POD shared under a
    spinlock, and each of those says why.
7.3 `std::formatter` takes the value's type: `volatile` needs a copy, `uint8_t`
    is a number. **MUST-FIX**
7.4 Parsing is strict — trailing junk, empty fields and wrong field counts are
    errors, not defaults.
7.5 Comments say *why*. Where a plausible alternative was rejected, name it and
    the number that decided it.

## 8. Concurrency

8.1 Who else touches this? State shared between tasks is either under the mux,
    atomic, or volatile and single-word plus monotonic.
8.2 Two readers of one stream: does the new code take bytes another reader
    wanted? **MUST-FIX**
8.3 A debounce or restore timer is long enough that a bouncing client cannot
    thrash the receiver before the restore finishes.

## 9. Docs and tests

9.1 `README.md` and `docs/` say what the change makes true, and stop saying what
    it made false.
9.2 New behaviour with a host-testable shape has a test (`tools/`, `unittest`,
    the framing suite). New behaviour without one says why in the commit.
9.3 New endpoints, fields or commands are in the docs with their ranges.
9.4 The commit message carries the evidence: what was run, what it returned,
    what surprised.

## 10. Anti-checklist — deliberate here, do not "fix"

- Exceptions and RTTI are off. `std::format` terminating on a bad format string
  is accepted.
- `char[]` at `lwIP`, `inet_ntop`, `httpd_req_recv` and `esp_ota_write`
  boundaries.
- No authentication on HTTP or tcp/3333: trusted LAN, by design.
- The bridge's synthesised MON-COMMS reply: the receiver ignores the message and
  u-center needs it.

## 11. Sign-off

| | |
|---|---|
| Commit | |
| Reviewer | |
| Design note (§0), or why not | |
| §2 exercises run | |
| Boots / uptime / heap after | |
| Deferred checks (§3.4) and trigger | |
| Findings, by section | |
| **JUDGEMENT** calls accepted, and why | |

---

### Worked example

`3c0cdd9` "GPS: save the antenna delay so it outlives the boot". §5.1 asks
whether the setter is paired with `save()` — the first version of that commit was
not, and the value read back correctly until the next boot, at which point the
boot's TP5 re-assert put the pulse back on an uncompensated figure. The setting
looked right the whole time and only the reboot test (§5.2) found it.
