use core::{
    borrow, cmp,
    ffi::c_void,
    hash::{Hash, Hasher},
    marker::PhantomData,
    mem,
    ops::Deref,
    ptr,
    sync::atomic::{AtomicUsize, Ordering},
};

extern crate alloc;

use alloc::boxed::Box;

/// A soft limit on the amount of references that may be made to an `Arc`.
///
/// Going above this limit will abort your program (although not
/// necessarily) at _exactly_ `MAX_REFCOUNT + 1` references.
const MAX_REFCOUNT: usize = (isize::MAX) as usize;

// `no_std`-compatible abort by forcing a panic while already panicing.
#[cfg(not(feature = "std"))]
#[cold]
fn abort() -> ! {
    struct PanicOnDrop;
    impl Drop for PanicOnDrop {
        fn drop(&mut self) {
            panic!()
        }
    }
    let _double_panicer = PanicOnDrop;
    panic!();
}

pub struct ArcInner<T: ?Sized> {
    pub(crate) count: AtomicUsize,
    pub(crate) data: T,
}

unsafe impl<T: ?Sized + Sync + Send> Send for ArcInner<T> {}
unsafe impl<T: ?Sized + Sync + Send> Sync for ArcInner<T> {}

pub struct Arc<T: ?Sized> {
    pub(crate) p: ptr::NonNull<ArcInner<T>>,
    pub(crate) phantom: PhantomData<T>,
}

unsafe impl<T: ?Sized + Sync + Send> Send for Arc<T> {}
unsafe impl<T: ?Sized + Sync + Send> Sync for Arc<T> {}

impl<T> Arc<T> {
    /// Construct an `Arc<T>`
    #[inline]
    pub fn new(data: T) -> Self {
        let ptr = Box::into_raw(Box::new(ArcInner {
            count: AtomicUsize::new(1),
            data,
        }));

        unsafe {
            Arc {
                p: ptr::NonNull::new_unchecked(ptr),
                phantom: PhantomData,
            }
        }
    }

    /// Convert the Arc<T> to a raw pointer, suitable for use across FFI
    ///
    /// Note: This returns a pointer to the data T, which is offset in the allocation.
    ///
    /// It is recommended to use OffsetArc for this.
    #[inline]
    pub fn into_raw(this: Self) -> *const T {
        let ptr = this.as_ptr();
        mem::forget(this);
        ptr
    }

    /// Returns the raw pointer.
    ///
    /// Same as into_raw except `self` isn't consumed.
    #[inline]
    pub fn as_ptr(&self) -> *const T {
        // SAFETY: This cannot go through a reference to `data`, because this method
        // is used to implement `into_raw`. To reconstruct the full `Arc` from this
        // pointer, it needs to maintain its full provenance, and not be reduced to
        // just the contained `T`.
        unsafe { ptr::addr_of_mut!((*self.ptr()).data) }
    }

    /// Returns the address on the heap of the Arc itself -- not the T within it -- for memory
    /// reporting.
    pub fn heap_ptr(&self) -> *const c_void {
        self.p.as_ptr() as *const ArcInner<T> as *const c_void
    }
}

impl<T: ?Sized> Arc<T> {
    /// Construct an `Arc` from an allocated `ArcInner`.
    /// # Safety
    /// The `ptr` must point to a valid instance, allocated by an `Arc`. The reference could will
    /// not be modified.
    unsafe fn from_raw_inner(ptr: *mut ArcInner<T>) -> Self {
        Arc {
            p: ptr::NonNull::new_unchecked(ptr),
            phantom: PhantomData,
        }
    }

    #[inline]
    pub(crate) fn inner(&self) -> &ArcInner<T> {
        // This unsafety is ok because while this arc is alive we're guaranteed
        // that the inner pointer is valid. Furthermore, we know that the
        // `ArcInner` structure itself is `Sync` because the inner data is
        // `Sync` as well, so we're ok loaning out an immutable pointer to these
        // contents.
        unsafe { &*self.ptr() }
    }

    // Non-inlined part of `drop`. Just invokes the destructor.
    #[inline(never)]
    unsafe fn drop_slow(&mut self) {
        let _ = Box::from_raw(self.ptr());
    }

    /// Test pointer equality between the two Arcs, i.e. they must be the _same_
    /// allocation
    #[inline]
    pub fn ptr_eq(this: &Self, other: &Self) -> bool {
        this.ptr() == other.ptr()
    }

    pub(crate) fn ptr(&self) -> *mut ArcInner<T> {
        self.p.as_ptr()
    }
}

impl<T: ?Sized> Clone for Arc<T> {
    #[inline]
    fn clone(&self) -> Self {
        // Using a relaxed ordering is alright here, as knowledge of the
        // original reference prevents other threads from erroneously deleting
        // the object.
        //
        // As explained in the [Boost documentation][1], Increasing the
        // reference counter can always be done with memory_order_relaxed: New
        // references to an object can only be formed from an existing
        // reference, and passing an existing reference from one thread to
        // another must already provide any required synchronization.
        //
        // [1]: (www.boost.org/doc/libs/1_55_0/doc/html/atomic/usage_examples.html)
        let old_size = self.inner().count.load(Ordering::Acquire);
        self.inner().count.store(old_size + 1, Ordering::Release);

        // However we need to guard against massive refcounts in case someone
        // is `mem::forget`ing Arcs. If we don't do this the count can overflow
        // and users will use-after free. We racily saturate to `isize::MAX` on
        // the assumption that there aren't ~2 billion threads incrementing
        // the reference count at once. This branch will never be taken in
        // any realistic program.
        //
        // We abort because such a program is incredibly degenerate, and we
        // don't care to support it.
        if old_size > MAX_REFCOUNT {
            abort();
        }

        unsafe {
            Arc {
                p: ptr::NonNull::new_unchecked(self.ptr()),
                phantom: PhantomData,
            }
        }
    }
}

impl<T: ?Sized> Deref for Arc<T> {
    type Target = T;

    #[inline]
    fn deref(&self) -> &T {
        &self.inner().data
    }
}

impl<T: Clone + ?Sized> Arc<T> {
    /// Makes a mutable reference to the `Arc`, cloning if necessary
    ///
    /// This is functionally equivalent to [`Arc::make_mut`][mm] from the standard library.
    ///
    /// If this `Arc` is uniquely owned, `make_mut()` will provide a mutable
    /// reference to the contents. If not, `make_mut()` will create a _new_ `Arc`
    /// with a copy of the contents, update `this` to point to it, and provide
    /// a mutable reference to its contents.
    ///
    /// This is useful for implementing copy-on-write schemes where you wish to
    /// avoid copying things if your `Arc` is not shared.
    ///
    /// [mm]: https://doc.rust-lang.org/stable/std/sync/struct.Arc.html#method.make_mut
    #[inline]
    pub fn make_mut(this: &mut Self) -> &mut T {
        if !this.is_unique() {
            // Another pointer exists; clone
            *this = Arc::new((**this).clone());
        }

        unsafe {
            // This unsafety is ok because we're guaranteed that the pointer
            // returned is the *only* pointer that will ever be returned to T. Our
            // reference count is guaranteed to be 1 at this point, and we required
            // the Arc itself to be `mut`, so we're returning the only possible
            // reference to the inner data.
            &mut (*this.ptr()).data
        }
    }
}

impl<T: ?Sized> Arc<T> {
    /// Provides mutable access to the contents _if_ the `Arc` is uniquely owned.
    #[inline]
    pub fn get_mut(this: &mut Self) -> Option<&mut T> {
        if this.is_unique() {
            unsafe {
                // See make_mut() for documentation of the threadsafety here.
                Some(&mut (*this.ptr()).data)
            }
        } else {
            None
        }
    }

    /// Whether or not the `Arc` is uniquely owned (is the refcount 1?).
    pub fn is_unique(&self) -> bool {
        // See the extensive discussion in [1] for why this needs to be Acquire.
        //
        // [1] https://github.com/servo/servo/issues/21186
        Self::count(self) == 1
    }

    /// Gets the number of [`Arc`] pointers to this allocation
    pub fn count(this: &Self) -> usize {
        this.inner().count.load(Ordering::Acquire)
    }
}

impl<T: ?Sized> Drop for Arc<T> {
    #[inline]
    fn drop(&mut self) {
        // Because `fetch_sub` is already atomic, we do not need to synchronize
        // with other threads unless we are going to delete the object.
        let count = self.inner().count.load(Ordering::Acquire);
        self.inner().count.store(count - 1, Ordering::Release);

        if count != 1 {
            return;
        }

        // FIXME(bholley): Use the updated comment when [2] is merged.
        //
        // This load is needed to prevent reordering of use of the data and
        // deletion of the data.  Because it is marked `Release`, the decreasing
        // of the reference count synchronizes with this `Acquire` load. This
        // means that use of the data happens before decreasing the reference
        // count, which happens before this load, which happens before the
        // deletion of the data.
        //
        // As explained in the [Boost documentation][1],
        //
        // > It is important to enforce any possible access to the object in one
        // > thread (through an existing reference) to *happen before* deleting
        // > the object in a different thread. This is achieved by a "release"
        // > operation after dropping a reference (any access to the object
        // > through this reference must obviously happened before), and an
        // > "acquire" operation before deleting the object.
        //
        // [1]: (www.boost.org/doc/libs/1_55_0/doc/html/atomic/usage_examples.html)
        // [2]: https://github.com/rust-lang/rust/pull/41714
        self.inner().count.load(Ordering::Acquire);

        unsafe {
            self.drop_slow();
        }
    }
}

impl<T: ?Sized + PartialEq> PartialEq for Arc<T> {
    fn eq(&self, other: &Arc<T>) -> bool {
        Self::ptr_eq(self, other) || *(*self) == *(*other)
    }

    #[allow(clippy::partialeq_ne_impl)]
    fn ne(&self, other: &Arc<T>) -> bool {
        !Self::ptr_eq(self, other) && *(*self) != *(*other)
    }
}

impl<T: ?Sized + PartialOrd> PartialOrd for Arc<T> {
    fn partial_cmp(&self, other: &Arc<T>) -> Option<cmp::Ordering> {
        (**self).partial_cmp(&**other)
    }

    fn lt(&self, other: &Arc<T>) -> bool {
        *(*self) < *(*other)
    }

    fn le(&self, other: &Arc<T>) -> bool {
        *(*self) <= *(*other)
    }

    fn gt(&self, other: &Arc<T>) -> bool {
        *(*self) > *(*other)
    }

    fn ge(&self, other: &Arc<T>) -> bool {
        *(*self) >= *(*other)
    }
}

impl<T: ?Sized + Ord> Ord for Arc<T> {
    fn cmp(&self, other: &Arc<T>) -> cmp::Ordering {
        (**self).cmp(&**other)
    }
}

impl<T: ?Sized + Eq> Eq for Arc<T> {}

impl<T: Default> Default for Arc<T> {
    #[inline]
    fn default() -> Arc<T> {
        Arc::new(Default::default())
    }
}

impl<T: ?Sized + Hash> Hash for Arc<T> {
    fn hash<H: Hasher>(&self, state: &mut H) {
        (**self).hash(state)
    }
}

impl<T> From<T> for Arc<T> {
    #[inline]
    fn from(t: T) -> Self {
        Arc::new(t)
    }
}

impl<T: ?Sized> borrow::Borrow<T> for Arc<T> {
    #[inline]
    fn borrow(&self) -> &T {
        &**self
    }
}

impl<T: ?Sized> AsRef<T> for Arc<T> {
    #[inline]
    fn as_ref(&self) -> &T {
        &**self
    }
}
