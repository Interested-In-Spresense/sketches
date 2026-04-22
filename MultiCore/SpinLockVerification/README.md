# SpinLockVerification

Spresense multi-core lock stress sample.

This sample is intended to be used mainly from the Arduino IDE.

## Purpose

This sample intentionally introduces long `delay()` calls while holding a shared spinlock.
MainCore and SubCore1..SubCore5 continuously compete for the same lock.

If the spinlock works, `collisions` should stay `0` while counters keep increasing.

## Files

- `MainCore/MainCore.ino`
- `Sub1Core/Sub1Core.ino`
- `Sub2Core/Sub2Core.ino`
- `Sub3Core/Sub3Core.ino`
- `Sub4Core/Sub4Core.ino`
- `Sub5Core/Sub5Core.ino`

## Arduino IDE Usage

Open each `.ino` file from the Arduino IDE and switch the board options before building and uploading.

Recommended upload order:

1. Open `Sub1Core/Sub1Core.ino`, set `Core` to `SubCore 1`, then upload.
2. Open `Sub2Core/Sub2Core.ino`, set `Core` to `SubCore 2`, then upload.
3. Open `Sub3Core/Sub3Core.ino`, set `Core` to `SubCore 3`, then upload.
4. Open `Sub4Core/Sub4Core.ino`, set `Core` to `SubCore 4`, then upload.
5. Open `Sub5Core/Sub5Core.ino`, set `Core` to `SubCore 5`, then upload.
6. Open `MainCore/MainCore.ino`, set `Core` to `MainCore`, set `Memory` to `640 KB`, then upload.
7. Open the Serial Monitor at `115200` baud.

## Expected monitor output

`[Main] collisions=0 owner=... entries(main/sub1/sub2/sub3/sub4/sub5)=... timeouts(main/sub1/sub2/sub3/sub4/sub5)=...`

- `entries_*` should increase over time.
- `collisions` should remain `0`.
- Some `timeouts_*` are acceptable under heavy contention.

## How to increase contention

Edit constants in each sketch:

- `HOLD_MS`
- `OUTER_DELAY_MS`
- `LOCK_SPIN_LIMIT`

Increasing `HOLD_MS` and lowering `LOCK_SPIN_LIMIT` will increase timeout pressure.

## MainCore Memory

When using the Arduino IDE, change the `Tools > Memory` setting for `MainCore` from the default value to `640 KB` before compiling and uploading.

This sample uses shared memory for the lock and the verification report, so lowering available memory on MainCore can affect startup behavior. Keep `MainCore` at `640 KB` when running this verification sketch.
