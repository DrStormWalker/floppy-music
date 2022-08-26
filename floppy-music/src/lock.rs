use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

use lock_api::{GuardSend, RawMutex};

pub struct RawTicketLock {
    next_ticket: AtomicUsize,
    next_serving: AtomicUsize,
}

unsafe impl RawMutex for RawTicketLock {
    const INIT: Self = Self {
        next_ticket: AtomicUsize::new(0),
        next_serving: AtomicUsize::new(0),
    };

    type GuardMarker = GuardSend;

    fn lock(&self) {
        let ticket = {
            let ticket = self.next_ticket.load(Ordering::Acquire);
            self.next_ticket.store(ticket + 1, Ordering::Release);
            ticket
        };

        while self.next_serving.load(Ordering::Acquire) != ticket {}
    }

    fn try_lock(&self) -> bool {
        let ticket = {
            let ticket = self.next_ticket.load(Ordering::Acquire);
            self.next_ticket.store(ticket + 1, Ordering::Release);
            ticket
        };

        self.next_serving.load(Ordering::Acquire) == ticket
    }

    unsafe fn unlock(&self) {
        let ticket = self.next_serving.load(Ordering::Acquire);
        self.next_serving.store(ticket + 1, Ordering::Release);
    }
}

pub type TicketLock<T> = lock_api::Mutex<RawTicketLock, T>;
pub type TicketLockGuard<'a, T> = lock_api::MutexGuard<'a, RawTicketLock, T>;

// 1. Define our raw lock type
pub struct RawSpinLock(AtomicBool);

// 2. Implement RawMutex for this type
unsafe impl RawMutex for RawSpinLock {
    const INIT: RawSpinLock = RawSpinLock(AtomicBool::new(false));

    // A spinlock guard can be sent to another thread and unlocked there
    type GuardMarker = GuardSend;

    fn lock(&self) {
        // Note: This isn't the best way of implementing a spinlock, but it
        // suffices for the sake of this example.
        while !self.try_lock() {}
    }

    fn try_lock(&self) -> bool {
        if !self.0.load(Ordering::Acquire) {
            self.0.store(true, Ordering::Release);
            true
        } else {
            false
        }
    }

    unsafe fn unlock(&self) {
        self.0.store(false, Ordering::Release);
    }
}

// 3. Export the wrappers. This are the types that your users will actually use.
pub type SpinLock<T> = lock_api::Mutex<RawSpinLock, T>;
pub type SpinLockGuard<'a, T> = lock_api::MutexGuard<'a, RawSpinLock, T>;
