package ca.team1310.swerve.utils;

import java.util.function.Supplier;

/**
 * Lightweight object that caches the result of a function for a specific number of milliseconds.
 * @author Tony Field
 * @since 2025-01-17 08:23
 */
public class Cache<T> {

    private final Supplier<T> supplier;
    private final long ttlMillis;
    private T value;
    private long fetchedTimestamp;

    /**
     * Create a new cache with the specified supplier and time-to-live.
     * @param supplier The supplier to fetch the value.
     * @param ttlMillis The time-to-live in milliseconds.
     */
    public Cache(Supplier<T> supplier, long ttlMillis) {
        this.supplier = supplier;
        this.ttlMillis = ttlMillis;
        this.value = null;
        this.fetchedTimestamp = 0;
    }

    /**
     * Get the cached value, fetching a new value if the cache is stale.
     * @return The cached value.
     */
    public T get() {
        long now = System.currentTimeMillis();
        if (value == null || now - fetchedTimestamp > ttlMillis) {
            value = supplier.get();
            fetchedTimestamp = now;
        }
        return value;
    }

    /**
     * Invalidate the cache, forcing a new value to be fetched on the next call to get().
     */
    public void invalidate() {
        fetchedTimestamp = 0;
        value = null;
    }
}
