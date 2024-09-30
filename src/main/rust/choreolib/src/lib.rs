use std::cell::Cell;

thread_local! {
    static CURRENT_JVM: Cell<*mut ()> = Cell::new(std::ptr::null_mut());
}
