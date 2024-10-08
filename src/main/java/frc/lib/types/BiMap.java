package frc.lib.types;

import java.util.HashMap;
import java.util.Map;

/** 1-to-1 map that is reversible. */
public class BiMap<K, V> {

    private transient Map<K, V> normalMap;
    private transient Map<V, K> reverseMap;

    public BiMap(final Map<K, V> normalMap, final Map<V, K> reverseMap) {
        this.normalMap = normalMap;
        this.reverseMap = reverseMap;
    }

    public BiMap() {
        this(new HashMap<>(), new HashMap<>());
    }

    public V get(K key) {
        return normalMap.get(key);
    }

    public K getKey(V value) {
        return reverseMap.get(value);
    }

    public void insert(K key, V value) {
        normalMap.put(key, value);
        reverseMap.put(value, key);
    }

    public void remove(V value) {
        K key = reverseMap.get(value);
        reverseMap.remove(value);
        normalMap.remove(key);
    }

    public void removeKey(K key) {
        V value = normalMap.get(key);
        reverseMap.remove(value);
        normalMap.remove(key);
    }
}
