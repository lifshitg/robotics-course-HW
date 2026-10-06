-- Permit line breaks in inline topic/type names, including narrow table cells.
function Code(el)
  local s = el.text:gsub('([%%#$&_{}])', '\\%1')
  return pandoc.RawInline('latex', '\\texttt{\\small\\seqsplit{' .. s .. '}}')
end
