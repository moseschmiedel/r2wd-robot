#[repr(i8)]
#[derive(PartialEq, Eq, PartialOrd, Ord, Debug, Clone)]
enum OccupationState {
    Occupied = 2,
    MaybeOccupied = 1,
    Unknown = 0,
    MaybeFree = -1,
    Free = -2,
}

struct MapCell {
    occupied: OccupationState,
}

struct Map {
    cells: Vec<MapCell>,
    dimension: (usize, usize),
}

impl Map {
    /// Construct a new Map with dimension (x_dim, y_dim).
    /// Pay attention to the dimensions of the Map. The size
    /// x_dim * y_dim * sizeof(MapCell) should fit in the memory.
    fn new(x_dim: usize, y_dim: usize) -> Self {
        Self {
            cells: Vec::with_capacity(x_dim * y_dim),
            dimension: (x_dim, y_dim),
        }
    }
}
