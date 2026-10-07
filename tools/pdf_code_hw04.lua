-- Preserve spaces in inline commands while allowing long tokens to wrap.
function Code(el)
  local s = el.text:gsub('%S+', function(word)
    local escaped = word:gsub('([%%#$&_{}])', '\\%1')
    return '\\seqsplit{' .. escaped .. '}'
  end)
  return pandoc.RawInline('latex', '\\texttt{\\small ' .. s .. '}')
end
