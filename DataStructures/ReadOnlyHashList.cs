using System.Collections;
using System.Collections.Generic;

namespace Jitter.DataStructures
{
    public class ReadOnlyHashList<T> : IEnumerable, IEnumerable<T>
    {
        private HashList<T> hashList;

        public ReadOnlyHashList(HashList<T> hashList)
        {
            this.hashList = hashList;
        }

        public IEnumerator GetEnumerator()
        {
            return hashList.GetEnumerator();
        }

        IEnumerator<T> IEnumerable<T>.GetEnumerator()
        {
            return hashList.GetEnumerator();
        }

        public int Count
        {
            get { return hashList.Count; }
        }

        public bool Contains(T item)
        {
            return hashList.Contains(item);
        }
    }
}