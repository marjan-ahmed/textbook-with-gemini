# Data Model: Local Search for Docusaurus Documentation

## Entities

### Documentation Page

Represents a single content unit within the Docusaurus site that is subject to indexing.

**Attributes**:
-   `id`: Unique identifier for the page (generated internally).
-   `title`: The title of the documentation page (string).
-   `content`: The full text content of the page, stripped of Markdown/MDX formatting, used for indexing (string).
-   `url`: The relative URL path to the documentation page (string).
-   `category`: The category or section the page belongs to (string, optional).
-   `tags`: A list of tags associated with the page (array of strings, optional).
-   `lastModified`: Timestamp of the last modification (number, optional).

**Relationships**:
-   A `Documentation Page` is indexed into one or more `Search Index` entries.

### Search Index

A processed, optimized data structure containing terms and references to `Documentation Page` entities, enabling fast full-text search. This is typically a JSON file generated during the Docusaurus build process.

**Attributes**:
-   `indexedTerms`: A collection of words and phrases extracted from `Documentation Page` content, often with associated weights or frequencies (object/array, structure dependent on search library).
-   `pageReferences`: Mappings from indexed terms back to the `id` of `Documentation Page` entities, including snippets or contextual information (object/array).
-   `metadata`: Configuration or versioning information for the index (object, optional).

**Relationships**:
-   References multiple `Documentation Page` entities.
